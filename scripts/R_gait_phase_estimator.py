#!/usr/bin/env python3

import rospy, rospkg, rosbag
from std_msgs.msg import Float64, Int16
import os
#os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' #Remove warnings for GPU (TODO: Review if it affect the algorithm)
import numpy as np
import csv
from sklearn.ensemble import GradientBoostingRegressor, RandomForestRegressor, VotingRegressor
from sklearn.linear_model import LinearRegression
from sklearn.tree import DecisionTreeRegressor
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split
import numpy as np
import joblib
from collections import deque

class GaitPhaseEstimator:
    def __init__(self):
        # ROS setup
        rospy.init_node('gait_phase_estimator', anonymous=True)

        # Paths to models and training data
        self.patient = rospy.get_param("patient", "daniel_train")
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('ankle_exoskeleton')

        self.labels_path = os.path.join(package_path, "log", "learning_models", f"{self.patient}_labels.csv")
        self.model_path = os.path.join(package_path, "log", "learning_models", f"{self.patient}_modelR.pkl")
        self.bag_path = os.path.join(package_path, "log", "training_bags", f"{self.patient}.bag")

        # Variables to track state
        self.modelLoaded = False
        self.model = []
        self.angleUpdated = False
        self.forceUpdated = False

        # Variables for derivatives
        self.last_angle_timestamp = None
        self.last_force_timestamp = None

        #Variable for estimation
        self.samples_size = 2
        self.data_sequence = deque(maxlen=self.samples_size)
        self.ankle_angle = None
        self.ground_force = None
        self.smoothed_estimated_phase = 0

        self.ankle_angle_sub = rospy.Subscriber('/ankle_joint/angle', Float64, self.ankle_angle_callback)
        self.ground_force_sub = rospy.Subscriber('/vGRF', Float64, self.ground_force_callback)
        self.gait_ptg_pub = rospy.Publisher('/gait_percentage_R', Int16, queue_size=2)
        self.force_dt_pub = rospy.Publisher('/ground_force_derivative', Float64, queue_size=2)
        self.angle_dt_pub = rospy.Publisher('/angle_force_derivative', Float64, queue_size=2)


    def ankle_angle_callback(self, msg):
        current_angle = msg.data

        if self.ankle_angle is not None:
            self.ankle_angle_derivative = self.calculate_derivative(current_angle, self.ankle_angle)
        else:
            self.ankle_angle_derivative = 0  # Initial derivative or no previous data

        self.ankle_angle = current_angle
        self.angleUpdated = True

    def ground_force_callback(self, msg):
        current_force = msg.data
        
        if self.ground_force is not None:
            self.ground_force_derivative = self.calculate_derivative(current_force, self.ground_force)
        else:
            self.ground_force_derivative = 0  # Initial derivative or no previous data

        self.ground_force = current_force
        self.forceUpdated = True
    
    def calculate_derivative(self, current_value, previous_value):
        """Calculate the derivative using finite difference method."""
        return (current_value - previous_value)

    def offline_phase_estimator(self, time, interpolated_forces):
        # Detect stance and swing phases based on force data
        stance_mask = interpolated_forces > 0.1  # True for stance, False for swing

        # Initialize phase and progress arrays
        gait_phases = []
        gait_progress = []

        # Track indices where each stance and swing phase begins and ends
        start_index = 0
        in_stance_phase = stance_mask[0]
        phase_boundaries = []

        # Identify boundaries of each stance and swing phase
        for i in range(1, len(stance_mask)):
            if stance_mask[i] != in_stance_phase:
                phase_boundaries.append((start_index, i - 1, in_stance_phase))
                start_index = i
                in_stance_phase = stance_mask[i]

        # Add the last boundary
        phase_boundaries.append((start_index, len(stance_mask) - 1, in_stance_phase))

        # Find complete gait cycles (stance + swing phase pairs)
        gait_cycles = []
        i = 0
        while i < len(phase_boundaries) - 1:
            # Ensure a stance phase is followed by a swing phase for a complete gait cycle
            start_stance, end_stance, is_stance = phase_boundaries[i]
            start_swing, end_swing, is_swing = phase_boundaries[i + 1]
            if is_stance and not is_swing:
                gait_cycles.append((start_stance, end_stance, start_swing, end_swing))  # Start of stance to end of swing
                i += 2  # Skip to the next possible cycle
            else:
                i += 1  # Continue looking for a complete cycle

        # Discard the first and last complete gait cycles
        if len(gait_cycles) > 2:
            gait_cycles = gait_cycles[1:-1]  # Remove first and last cycles

        start_time = time[gait_cycles[0][0]]
        end_time = time[gait_cycles[-1][3]]
        # Assign gait phases and progress for the remaining cycles
        for start_stance, end_stance, start_swing, end_swing in gait_cycles:
            # Identify if this is a stance or swing phase
            stance_duration = start_swing - start_stance
            swing_duration = end_swing - end_stance

            # 60% for stance phase
            stance_progress = np.linspace(0, 60, stance_duration, endpoint=False)
            swing_progress = np.linspace(60, 100, swing_duration, endpoint=False)

            gait_phases.extend(['stance_phase'] * stance_duration)
            gait_phases.extend(['swing_phase'] * swing_duration)
            gait_progress.extend(stance_progress)
            gait_progress.extend(swing_progress)

        # Convert lists to numpy arrays for easier processing
        gait_phases = np.array(gait_phases)
        gait_progress = np.array(gait_progress)

        #gait_progress = np.round(gait_progress / 2) * 2
        #gait_progress[gait_progress > 98] = 0

        return gait_phases, gait_progress, start_time, end_time
        
    def train_model(self):
        rospy.loginfo(f"Training model for patient {self.patient}...")

        # Check if the bag file exists
        if not os.path.exists(self.bag_path):
            rospy.logerr(f"No .bag file found for patient {self.patient} in {self.bag_path}. Training cannot proceed.")
            rospy.signal_shutdown("Missing .bag file for training.")
            return

        # Read bag file and extract data
        bag = rosbag.Bag(self.bag_path)
        angle_data = []
        vgrf_data = []

        for topic, msg, t in bag.read_messages(topics=['/ankle_joint/angle', '/vGRF']):
            if topic == '/ankle_joint/angle':
                angle_data.append((t.to_sec(), msg.data))
            elif topic == '/vGRF':
                vgrf_data.append((t.to_sec(), msg.data))

        bag.close()

        # Convert to numpy arrays
        angle_data = np.array(angle_data)
        vgrf_data = np.array(vgrf_data)

        # Interpolate angle data to match vGRF timestamps
        interpolated_angles = np.interp(vgrf_data[:, 0], angle_data[:, 0], angle_data[:, 1])
        interpolated_forces = vgrf_data[:, 1]
        time = vgrf_data[:, 0] - vgrf_data[0, 0]

        # Derivatives of force and angle
        force_derivatives = np.gradient(interpolated_forces)
        angle_derivatives = np.gradient(interpolated_angles)
        
        # Gait phase detector offline
        (gait_phases, gait_progress, start_time, end_time) = self.offline_phase_estimator(time, interpolated_forces)

        #Adjust Data to the Gait Phase anaylized (without the first and last cycle)
        mask = (time >= start_time) & (time <= end_time)
        adjusted_time = time[mask]
        adjusted_force = interpolated_forces[mask]
        adjusted_angle = interpolated_angles[mask]
        adjusted_force_derivatives = force_derivatives[mask]
        adjusted_angle_derivatives = angle_derivatives[mask]
        
        # --- Write data to CSV ---
        rospy.loginfo(f"Saving gait data to {self.labels_path}...")
        with open(self.labels_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'Force', 'Force_Derivative', 'Angle', 'Angle_Derivative', 'Gait_Progress', 'Phase'])

            for i in range(len(gait_phases)):
                writer.writerow([
                    adjusted_time[i],  # Time
                    adjusted_force[i],  # Force
                    adjusted_force_derivatives[i],  # Force derivative
                    adjusted_angle[i],  # Angle
                    adjusted_angle_derivatives[i],  # Angle derivative
                    gait_progress[i],  # Gait progress
                    gait_phases[i]  # Phase
                ])

        rospy.loginfo(f"Gait data saved successfully.")

        # Feature selection: Force, Force_Derivative, Angle, Angle_Derivative
        X = np.column_stack((adjusted_force, adjusted_force_derivatives, adjusted_angle, adjusted_angle_derivatives))
        y = np.array(gait_progress)

        # Split data into training and test sets
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
        

        # Initialize and train the model
        # Initialize models
        model1 = GradientBoostingRegressor(n_estimators=100, learning_rate=0.1, max_depth=9, random_state=42)
        model2 = RandomForestRegressor(n_estimators=100, random_state=42)
        model3 = VotingRegressor(estimators=[
            ('lr', LinearRegression()),
            ('rf', RandomForestRegressor(n_estimators=100, random_state=42)),
            ('gb', GradientBoostingRegressor(n_estimators=100, learning_rate=0.1, max_depth=9, random_state=42))
        ])

        # Dictionary to store models and their names
        models = {'Gradient Boosting': model1, 'Random Forest': model2, 'Voting Regressor': model3}
        mse_results = {}

        # Train and evaluate each model
        for name, model in models.items():
            # Train the model
            model.fit(X_train, y_train)
            
            # Predict on test set
            y_pred = model.predict(X_test)
            
            # Calculate MSE
            mse = mean_squared_error(y_test, y_pred)
            mse_results[name] = mse  # Store MSE in dictionary
            
            # Print MSE for each model
            print(f"{name} Mean Squared Error: {mse:.4f}")

        # Choose a model to save (e.g., the model with the lowest MSE)
        best_model_name = min(mse_results, key=mse_results.get)
        model = models[best_model_name]

        model.fit(X_train, y_train)

        # Evaluate model on test data
        y_pred = model.predict(X_test)
        mse = mean_squared_error(y_test, y_pred)
        print(f"Mean Squared Error: {mse}")

        # --- Save model in pkl format---
        rospy.loginfo(f"Saving model to {self.model_path}...")
        joblib.dump(model, self.model_path)
        rospy.loginfo(f"Model saved successfully.")
        # Final of Training code

        # Load the learning model
        try:
            self.model = joblib.load(self.model_path)
            self.modelLoaded = True
            return 1
        except FileNotFoundError:
            rospy.logerr("Model was not loaded, please verify")
            self.modelLoaded = False
            return 0
    
    def mean_filter(self, predictions, window_size):
        return np.convolve(predictions, np.ones(window_size)/window_size, mode='same')


    def estimate_phase(self):
        if (self.modelLoaded is not None) and (self.angleUpdated == True) and (self.forceUpdated == True):
            # Estimation code
            current_input = [self.ground_force, self.ground_force_derivative, self.ankle_angle, self.ankle_angle_derivative]

            # FOR TEST
            self.angle_dt_pub.publish(self.ankle_angle_derivative)
            self.force_dt_pub.publish(self.ground_force_derivative)

            # Add the current input to the deque, which maintains the last N timesteps
            self.data_sequence.append(current_input)
            
            self.current_phase = 0
            
            if len(self.data_sequence) == self.samples_size:
                # Convert deque to numpy array and reshape for model input
                X = np.array(self.data_sequence)

                # Perform prediction
                estimated_phase = self.model.predict(X)

                # Update current phase with the adjusted estimated phase
                self.current_phase = estimated_phase

                self.smoothed_estimated_phase = self.mean_filter(estimated_phase, self.samples_size)[-1]

                self.gait_ptg_pub.publish(int(self.smoothed_estimated_phase))

            # Reset flags
            self.angleUpdated = False
            self.forceUpdated = False


    def run(self):
        rate = rospy.Rate(200)  # Set the frequency to 200 Hz

        if os.path.exists(self.model_path):
            rospy.loginfo(f"Model found for patient {self.patient}. Proceeding with phase estimation.")
            self.model = joblib.load(self.model_path)
            self.modelLoaded = True
        else:
            rospy.logwarn(f"Model not found for patient {self.patient}. Training a new model.")
            res = self.train_model()
            if res == 0:
                rospy.signal_shutdown("Model was not found")
        
        # Main loop running at 200 Hz
        rospy.loginfo(f"Estimating phase for patient {self.patient} using model {self.model_path}...")
        while not rospy.is_shutdown():
            if self.modelLoaded:
                self.estimate_phase()  

            rate.sleep()

if __name__ == '__main__':
    try:
        estimator = GaitPhaseEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass