#!/usr/bin/env python
import rospy
import rospkg
import pickle
import numpy as np
import sys
from sklearn.preprocessing import normalize
from pomegranate import*
from pomegranate.hmm import log as ln
from pomegranate import HiddenMarkovModel as HMM
from pomegranate import MultivariateGaussianDistribution as MGD
from scipy import io as scio
from scipy import linalg
from std_msgs.msg import Int8
from ankle_exoskeleton.msg import IMUData

"""Supported decoder algorithms"""
DECODER_ALGORITHMS = frozenset(("fov", "bvsw"))   # FOV: Forward-only viterbi, BVSW: Bounded Variable Sliding Window Approach

"""Real-time HMM class"""
class RealTimeHMM():
    def __init__(self, n_trials=3, leave_one_out=1):
        """Variable initialization"""
        self.patient = rospy.get_param("patient")
        self.verbose = rospy.get_param("verbose")
        self.n_trials = n_trials
        self.n_features = 2      # Raw data and 1st-derivative
        self.leave_one_out = leave_one_out
        self.rec_data = 0.0       # Number of recorded IMU data
        self.proc_data = 0.0      # Number of extracted features
        self.raw_win = [None] * 3
        self.ff = [[] for x in range(self.n_trials)]      # Training and test dataset
        self.labels = [[] for x in range(self.n_trials)]  # Reference labels from local data
        self.first_eval = True
        self.model_loaded = False
        algorithm = rospy.get_param("algorithm")
        print algorithm
        if algorithm not in DECODER_ALGORITHMS:
            raise ValueError("Unknown decoder {!r}".format(algorithm))
        self.decode = {
            "fov": self._run_fov,
            "bvsw": self._run_bvsw
        }[algorithm]
        self.imu_callback = {
            "fov": self._fov_callback,
            "bvsw": self._bvsw_callback
        }[algorithm]
        """HMM variables"""
        ''' State list:
            s1: Heel Strike (HS)
            s2: Flat Foot   (FF)
            s3: Heel Off    (HO)
            s4: Swing Phase (SP)'''
        self.model_name = "Gait"
        self.states = ['s1', 's2', 's3', 's4']
        self.n_states = len(self.states)
        self.state2phase = {"s1": "hs", "s2": "ff", "s3": "ho", "s4": "sp"}
        self.mgds = {}
        self.dis_means = [[] for x in range(self.n_states)]
        self.dis_covars = [[] for x in range(self.n_states)]
        self.start_prob = [1.0/self.n_states]*self.n_states
        self.trans_mat = np.zeros([self.n_states, self.n_states])
        self.log_startprob = []
        self.log_transmat = np.empty((self.n_states, self.n_states))
        self.max_win_len = 11       # ms (120 ms: mean IC duration for healthy subjects walking at comfortable speed)
        self.viterbi_path = np.empty((self.max_win_len+1, self.n_states))
        self.backtrack = [[None for x in range(self.n_states)] for y in range(self.max_win_len+1)]
        self.global_path = []
        self.work_buffer = np.empty(self.n_states)
        self.boundary = 1
        self.buff_len = 0
        self.states_pos = {}
        for i in range(len(self.states)):
            self.states_pos[self.states[i]] = i
        self.last_state = -1
        self.curr_state = -1
        self.conv_point = 0
        self.conv_found = False
        self.smp_freq = 100.0   # Hz
        self.fp_thresh = 1/self.smp_freq*8    # Threshold corresponds to 8 samples
        self.time_passed = 0.0
        self.obs = [[None for x in range(self.n_features)] for y in range(self.max_win_len)]
        self.model = HMM(name=self.model_name)
        """ROS init"""
        rospy.init_node('real_time_HMM', anonymous=True)
        rospack = rospkg.RosPack()
        self.packpath = rospack.get_path('ankle_exoskeleton')
        self.init_subs()
        self.init_pubs()
        """HMM-training (if no model exists)"""
        try:
            '''HMM-model loading'''
            with open(self.packpath+'/log/HMM_models/'+self.patient+'.txt') as infile:
                json_model = json.load(infile)
                self.model = HMM.from_json(json_model)
                rospy.logwarn(self.patient + "'s HMM model was loaded.")
        except IOError:
            """Local data loading and feature extraction"""
            self.load_data()
            rospy.logwarn("HMM model not trained yet!")
            self.init_hmm()
            self.train_hmm()
        try:
            '''MGDs loading'''
            for st in self.states:
                with open(self.packpath+'/log/HMM_models/'+self.patient+'_'+self.state2phase[st]+'.txt') as infile:
                    yaml_dis = yaml.safe_load(infile)
                    dis = MGD.from_yaml(yaml_dis)
                    self.mgds[st] = dis
                    rospy.logwarn(self.patient +"'s " + self.state2phase[st] + " MGC was loaded.")
                    '''Loading means and covariance matrix'''
                    self.dis_means[self.states_pos[st]] = self.mgds[st].parameters[0]
                    self.dis_covars[self.states_pos[st]] = self.mgds[st].parameters[1]
        except yaml.YAMLError as exc:
            rospy.logwarn("Not able to load distributions: " + exc)
        """Transition and initial (log) probabilities matrices upon training"""
        trans_mat = self.model.dense_transition_matrix()[:self.n_states,:self.n_states]
        for i in range(self.n_states):
            self.log_startprob.append(ln(self.start_prob[i]))
            for j in range(self.n_states):
                self.log_transmat[i,j] = ln(trans_mat[i][j])
        if self.verbose: print self.log_transmat
        self.model_loaded = True

    """Init ROS publishers"""
    def init_pubs(self):
        self.phase_pub = rospy.Publisher('/phase', Int8, queue_size=100)

    """Init ROS subcribers"""
    def init_subs(self):
        rospy.Subscriber('/imu_data', IMUData, self.imu_callback)

    """Callback function upon arrival of IMU data for forward-only decoding"""
    def _fov_callback(self, data):
        self.rec_data += 1.0
        self.raw_win.append(data.gyro_y)
        self.raw_win.pop(0)       # Drop first element

        if self.rec_data >= 3 and self.model_loaded:      # At least one previous and one subsequent data should have been received
            """Extract feature and append it to test dataset"""
            test_set = [self.raw_win[1], (self.raw_win[0]+self.raw_win[2])/2]    # First-derivate of angular velocity
            '''Forward-only decoding approach'''
            state = self.decode(test_set)
            self.time_passed += 1/self.smp_freq
            if self.last_state != state:
                self.last_state = state
                self.time_passed = 1/self.smp_freq
                rospy.logwarn("Detected phase: {}".format(self.state2phase[self.states[self.last_state]]))
            elif self.time_passed >= self.fp_thresh:
                if (self.curr_state == 3 and state == 0) or (state == self.curr_state + 1):
                    self.curr_state = state
                    self.phase_pub.publish(state)
            else:
                self.phase_pub.publish(self.curr_state)

    """Callback function upon arrival of IMU data for BVSW"""
    def _bvsw_callback(self, data):
        self.rec_data += 1.0
        self.raw_win.append(data.gyro_y)
        self.raw_win.pop(0)       # Drop first element

        if self.rec_data >= 3 and self.model_loaded:      # At least one previous and one subsequent data should have been received
            """Extract feature and append it to test dataset"""
            test_set = [self.raw_win[1], (self.raw_win[0]+self.raw_win[2])/2]    # First-derivate of angular velocity
            '''Bounded sliding window decoding approach'''
            self.obs.append(test_set)
            self.obs.pop(0)                     # This way, -1 element corresponds to last received features
            states = self.decode(test_set)
            if len(states) != 0:
                for st in states:
                    self.phase_pub.publish(st)
                    if self.curr_state != st:
                        rospy.logwarn("Detected phase: {}".format(self.state2phase[self.states[st]]))
                        self.curr_state = st
                self.proc_data += 1.0     # One gyro data has been processed

    """Local data loading and feature extraction"""
    def load_data(self):
        """Data loading"""
        datapath = self.packpath + "/log/mat_files/"
        data = [[] for x in range(self.n_trials)]
        for i in range(self.n_trials):
            data[i] = scio.loadmat(datapath + self.patient + "_proc_data" + str(i+1) + ".mat")
        gyro_y = [[] for x in range(self.n_trials)]
        time_array = [[] for x in range(self.n_trials)]
        for i in range(self.n_trials):
            gyro_y[i] = data[i]["gyro_y"][0]
            time_array[i] = data[i]["time"][0]
            self.labels[i] = data[i]["labels"][0]

        """Feature extraction"""
        '''First derivative'''
        fder_gyro_y = []
        for i in range(self.n_trials):
            der = []
            der.append(gyro_y[i][0])
            for j in range(1,len(gyro_y[i])-1):
                der.append((gyro_y[i][j+1]-gyro_y[i][j-1])/2)
                der.append(gyro_y[i][-1])
                fder_gyro_y.append(der)

        """Create training and test data"""
        for j in range(self.n_trials):
            for k in range(len(time_array[j])):
                f_ = []
                f_.append(gyro_y[j][k])
                f_.append(fder_gyro_y[j][k])
                self.ff[j].append(f_)
        self.ff = np.array(self.ff)
        self.n_features = len(self.ff[0][0])

    """Init HMM if no previous training"""
    def init_hmm(self):
        rospy.logwarn("-------Leaving trial {} out-------".format(self.leave_one_out+1))

        """Transition matrix (A)"""
        '''Transition matrix from reference labels'''
        prev = -1
        for i in range(len(self.labels[self.leave_one_out])):
            if prev == -1:
                prev = self.labels[self.leave_one_out][i]
            self.trans_mat[prev][self.labels[self.leave_one_out][i]] += 1.0
            prev = self.labels[self.leave_one_out][i]
        self.trans_mat = normalize(self.trans_mat, axis=1, norm='l1')
        '''Left-right model'''
        # self.trans_mat = np.array([(0.9, 0.1, 0, 0), (0, 0.9, 0.1, 0), (0, 0, 0.9, 0.1), (0.1, 0, 0, 0.9)])
        '''Right-left-right model'''
        # self.trans_mat = np.array([0.8, 0.1, 0, 0.1], [0.1, 0.8, 0.1, 0], [0, 0.1, 0.8, 0.1], [0.1, 0, 0.1, 0.8])
        if self.verbose: rospy.logwarn("TRANSITION MATRIX\n" + str(self.trans_mat))

        class_data = [[] for x in range(self.n_states)]
        for i in range(len(self.ff[self.leave_one_out])):
            class_data[self.labels[self.leave_one_out][i]].append(self.ff[self.leave_one_out][i])
        """Multivariate Gaussian Distributions for each hidden state"""
        class_means = [[[] for x in range(self.n_features)] for i in range(self.n_states)]
        class_vars = [[[] for x in range(self.n_features)] for i in range(self.n_states)]
        class_std = [[[] for x in range(self.n_features)] for i in range(self.n_states)]
        class_cov = []
        for i in range(self.n_states):
            cov = np.ma.cov(np.array(class_data[i]), rowvar=False)
            class_cov.append(cov)
            for j in range(self.n_features):
                class_means[i][j] = np.array(class_data[i][:])[:, [j]].mean(axis=0)
                class_vars[i][j] = np.array(class_data[i][:])[:, [j]].var(axis=0)
                class_std[i][j] = np.array(class_data[i][:])[:, [j]].std(axis=0)
        """Classifier initialization"""
        distros = []
        hmm_states = []
        for i in range(self.n_states):
            dis = MGD\
                (np.array(class_means[i]).flatten(),
                 np.array(class_cov[i]))
            st = State(dis, name=self.states[i])
            distros.append(dis)
            hmm_states.append(st)
        self.model.add_states(hmm_states)
        '''Initial transitions'''
        for i in range(self.n_states):
            self.model.add_transition(self.model.start, hmm_states[i], self.start_prob[i])
        '''Left-right model'''
        for i in range(self.n_states):
            for j in range(self.n_states):
                self.model.add_transition(hmm_states[i], hmm_states[j], self.trans_mat[i][j])
        '''Finish model setup'''
        self.model.bake()

    """Train initialized model"""
    def train_hmm(self):
        x_train = []
        for i in range(len(self.ff[self.leave_one_out-1])):
            x_train.append(self.ff[self.leave_one_out-1][i])
        for i in range(len(self.ff[(self.leave_one_out+1) % self.n_trials])):
            x_train.append(self.ff[(self.leave_one_out+1) % self.n_trials][i])
        x_train = list([x_train])
        rospy.logwarn("Training initialized model...")
        self.model.fit(x_train, algorithm='baum-welch', verbose=self.verbose)
        self.model.freeze_distributions()     # Freeze all model distributions, preventing update from ocurring
        '''Save Multivariate Gaussian Distributions into yaml file'''
        for st in self.model.states:
            if st.name != self.model_name+"-start" and st.name != self.model_name+"-end":
                dis = st.distribution
                dis_yaml = dis.to_yaml()
                try:
                    with open(self.packpath+'/log/HMM_models/'+self.patient+'_'+self.state2phase[st.name]+'.txt', 'w') as outfile:
                        yaml.dump(dis_yaml, outfile, default_flow_style=False)
                    rospy.logwarn(self.patient+"'s "+self.state2phase[st.name]+" distribution was saved.")
                except IOError:
                    rospy.logwarn('It was not possible to write HMM model.')
        '''Save model (json script) into txt file'''
        model_json = self.model.to_json()
        try:
            with open(self.packpath+'/log/HMM_models/'+self.patient+'.txt', 'w') as outfile:
                json.dump(model_json, outfile)
            rospy.logwarn(self.patient+"'s HMM model was saved.")
        except IOError:
            rospy.logwarn('It was not possible to write HMM model.')

    """Compute the log probability under a multivariate Gaussian distribution.
    Parameters
    ----------
        X : array_like, shape (n_samples, n_features)
            List of n_features-dimensional data points. Each row corresponds to a
            single data point.
        means : array_like, shape (n_components, n_features)
            List of n_features-dimensional mean vectors for n_components Gaussians.
            Each row corresponds to a single mean vector.
        covars : array_like
            List of n_components covariance parameters for each Gaussian. The shape
            is (n_components, n_features, n_features) if 'full'
        Returns
    ----------
        lpr : array_like, shape (n_samples, n_components)
            Array containing the log probabilities of each data point in
            X under each of the n_components multivariate Gaussian distributions."""
    def log_multivariate_normal_density(self, X, min_covar=1.e-7):
        """Log probability for full covariance matrices."""
        n_samples, n_dim = X.shape
        nmix = len(self.dis_means)
        log_prob = np.empty((n_samples, nmix))
        for c, (mu, cv) in enumerate(zip(self.dis_means, self.dis_covars)):
            try:
                cv_chol = linalg.cholesky(cv, lower=True)
            except linalg.LinAlgError:
                # The model is most probably stuck in a component with too
                # few observations, we need to reinitialize this components
                try:
                    cv_chol = linalg.cholesky(cv + min_covar * np.eye(n_dim),
                                              lower=True)
                except linalg.LinAlgError:
                    raise ValueError("'covars' must be symmetric, "
                                     "positive-definite")

            cv_log_det = 2 * np.sum(np.log(np.diagonal(cv_chol)))
            cv_sol = linalg.solve_triangular(cv_chol, (X - mu).T, lower=True).T
            log_prob[:, c] = - .5 * (np.sum(cv_sol ** 2, axis=1) +
                                     n_dim * np.log(2 * np.pi) + cv_log_det)
        return log_prob

    """Find argument (pos) that has the maximum value in array-like object"""
    def _argmax(self, X):
        X_max = float("-inf")
        pos = 0
        for i in range(X.shape[0]):
            if X[i] > X_max:
                X_max = X[i]
                pos = i
        return pos

    """Find max value in array-like object"""
    def _max(self, X):
        return X[self._argmax(X)]

    """Backtracking process to decode most-likely state sequence"""
    def _optim_backtrack(self, k):
        opt = []
        self.last_state = where_from = self._argmax(self.viterbi_path[k])
        opt.append(where_from)
        for lp in range(k-1, -1, -1):
            opt.insert(0, self.backtrack[lp + 1][where_from])
            where_from = self.backtrack[lp + 1][where_from]
        self.global_path.extend(opt)
        return opt

    '''Forward-only decoding approach'''
    def _run_fov(self, test_set):
        # Probability distribution of state given observation
        framelogprob = self.log_multivariate_normal_density(np.array([test_set]))
        if self.first_eval:
            for i in range(self.n_states):
                self.viterbi_path[0, i] = self.log_startprob[i] + framelogprob[0, i]
            self.first_eval = False
            return self._argmax(self.viterbi_path[0])
        else:       # Recursion
            for i in range(self.n_states):
                for j in range(self.n_states):
                    self.work_buffer[j] = (self.log_transmat[j, i] + self.viterbi_path[0, j])
                self.viterbi_path[1, i] = self._max(self.work_buffer) + framelogprob[0, i]
            self.viterbi_path[0] = self.viterbi_path[1]       # Prepare for next feature vector
            return self._argmax(self.viterbi_path[1])

    '''Bounded sliding variable window approach'''
    def _run_bvsw(self, test_set):
        framelogprob = self.log_multivariate_normal_density(np.array([test_set]))
        if self.first_eval:
            for i in range(self.n_states):
                self.viterbi_path[0,i] = self.log_startprob[i] + framelogprob[0,i]
                self.backtrack[0][i] = None
            self.first_eval = False
            return []
        else:
            '''Find likelihood probability and backpointer'''
            for j in range(self.n_states):
                for i in range(self.n_states):
                    self.work_buffer[i] = self.viterbi_path[self.boundary - 1][i] + self.log_transmat[i, j]
                self.viterbi_path[self.boundary][j] = self._max(self.work_buffer) + framelogprob[0][j]
                self.backtrack[self.boundary][j] = self._argmax(self.work_buffer)
            '''Backtracking local paths'''
            local_paths = [[] for x in range(self.n_states)]
            for j in range(self.n_states):
                where_from = j
                for smp in range(self.boundary-1, -1, -1):
                    local_paths[j].insert(0, self.backtrack[smp+1][where_from])
                    where_from = self.backtrack[smp+1][where_from]
            if self.verbose:
                print "\n{}, {}".format(t, b)
                for path in local_paths:
                    print path
            '''Given all local paths, find fusion point'''
            tmp = [None] * self.n_states
            for k in range(len(local_paths[0])-1, 0, -1):
                for st in range(self.n_states):
                    tmp[st] = local_paths[st][k]
                if tmp.count(tmp[0]) == len(tmp):      # All local paths point to only one state?
                    self.conv_found = True
                    self.conv_point = k
                    if self.verbose: print "Found, {}".format(k)
                    break
            '''Find local path if fusion point was found'''
            if self.boundary < self.max_win_len and self.conv_found:
                self.buff_len += self.conv_point
                opt = self._optim_backtrack(self.conv_point)
                self.conv_found = False
                if self.verbose: print "\nOpt1: " + str(opt) + ", {}".format(len(self.global_path))
                '''Reinitialize local variables'''
                for i in range(self.n_states):
                    if i == self.last_state:
                        self.log_startprob[i] = ln(1.0)
                    else:
                        self.log_startprob[i] = ln(0.0)
                    self.viterbi_path[0][i] = self.log_startprob[i] + framelogprob[0][i]
                    self.backtrack[0][i] = None
                for smp in range(1, self.boundary-self.conv_point+1):
                    for j in range(self.n_states):
                        for i in range(self.n_states):
                            self.work_buffer[i] = self.viterbi_path[smp - 1][i] + self.log_transmat[i, j]
                        self.viterbi_path[smp][j] = self._max(self.work_buffer) + self.log_multivariate_normal_density(np.array([self.obs[self.conv_point-self.boundary+smp-1]]))[0,j]
                        self.backtrack[smp][j] = self._argmax(self.work_buffer)
                self.boundary -= self.conv_point-1
                return opt
            elif self.boundary >= self.max_win_len:
                '''Bounding threshold was exceeded'''
                self.buff_len += self.max_win_len
                opt = self._optim_backtrack(self.boundary-1)
                if self.verbose: print "\nOpt2: " + str(opt) + ", {}".format(len(self.global_path))
                '''Reinitialize local variables'''
                self.boundary = 1
                for i in range(self.n_states):
                    if i == self.last_state:
                        self.log_startprob[i] = ln(1.0)
                    else:
                        self.log_startprob[i] = ln(0.0)
                    self.viterbi_path[0][i] = self.log_startprob[i] + framelogprob[0][i]
                    self.backtrack[0][i] = None
                return opt
            else:
                self.boundary += 1
                return []


def main():
    # if(len(sys.argv)<2):
    #     print("Missing patient's name.")
    #     exit()
    # else:
    #     patient = sys.argv[1]

    RtHMM = RealTimeHMM()
    rospy.logwarn("Spinning...")
    rospy.spin()
    print "Rata of processed data: " + str(RtHMM.proc_data/RtHMM.rec_data)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
