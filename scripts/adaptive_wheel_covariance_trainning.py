#!/usr/bin/env python3
import rospy
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, fabs, tan
from time import sleep
import tf
import sys
import roslib
import numpy as np
import copy
from std_msgs.msg import Time
from std_msgs.msg import String
import keras
from keras.models import Sequential
from keras.layers import Dense, Concatenate
from keras.models import load_model, Model
from keras.layers import Conv1D, Conv2D, MaxPooling2D, MaxPooling1D, Flatten, Concatenate, Input, InputLayer
import threading
import wave
import csv
import matplotlib.pyplot as plt
import struct
import sklearn
from sklearn.model_selection import KFold
from audio_common_msgs.msg import AudioDataStamped
from audio_common_msgs.msg import AudioData
from audio_common_msgs.msg import AudioInfo
from kuka_rsi_msgs.msg import FroniusState
from kuka_rsi_msgs.msg import Fronius500iState
from std_msgs.msg import Bool
import gi
gi.require_version('Gst', '1.0')
from std_msgs.msg import Float64
from gi.repository import Gst, GObject
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import pickle
import pandas as pd
from contextlib import redirect_stdout
from sklearn.model_selection import StratifiedKFold
from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import train_test_split
from audio_defect_tracking_msg.msg import audio_trackin_result
from rosgraph_msgs.msg import Clock

# exp_name = sys.argv[1]

# Define ANSI escape sequences for different colors
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
END = '\033[0m'

class audio_tracking():

    # constructor class
    def __init__(self):
        # variables
        # current time
        self.current_time = 0.0

        # booleans
        self.finish_training = False

        # audio parametes
        self.frame_rate = 0
        self.audio_dim = 0

        # vectors
        # airs_pressure
        self.inputs_audio_pressure = np.empty((0,))
        # power_spectrum of fft
        self.inputs_audio_fftpower = np.empty((0,))
        # phase sprectrum of fft
        self.inputs_audio_fftphase = np.empty((0,))
        # current sprectrum of fft
        self.inputs_current_data = np.empty((0,))
        # voltage sprectrum of fft
        self.inputs_voltage_data = np.empty((0,))
        # travel speed
        self.inputs_travel_speed_data = np.empty((0,))
        # wire feed speed
        self.inputs_wire_feed_speed_data = np.empty((0,))

        # concatenate data
        self.inputs_data = np.empty((0,))
        self.output_data = np.empty((0,))
        self.inputs_prediction_data = np.empty((0,))

        # training data
        self.inputs_audio_pressure_training = np.empty((0,))
        self.inputs_audio_fftpower_training = np.empty((0,))
        self.inputs_audio_fftphase_training = np.empty((0,))
        self.inputs_current_data_training = np.empty((0,))
        self.inputs_voltage_data_training = np.empty((0,))
        self.iaudio_namenputs_travel_speed_data_training = np.empty((0,))
        self.inputs_wire_feed_speed_data_training = np.empty((0,))
        self.output_training = np.empty((0,))

        self.inputs_audio_pressure_test = np.empty((0,))
        self.inputs_audio_fftpower_test = np.empty((0,))
        self.inputs_audio_fftphase_test = np.empty((0,))
        self.inputs_current_data_test = np.empty((0,))
        self.inputs_voltage_data_test = np.empty((0,))
        self.inputs_travel_speed_data_test = np.empty((0,))
        self.inputs_wire_feed_speed_data_test = np.empty((0,))
        self.output_test = np.empty((0,))

        self.predict_model = ""       # audio parameters
        self.channels = 0             # Number of channels
        self.sample_rate = 0.0        # Sampling rate [Hz]
        self.sample_format = "A"      # Audio format (e.g. S16LE)
        self.bitrate = 0.0            # Amount of audio data per second [bits/s]
        self.coding_format = "mp3"    # Audio coding format (e.g. WAVE, MP3)
        self.InfoRead = False         # Boolean

        self.audio_buffer = np.empty((0,2))        # audio buffer with timestamp in seconds
        self.current_buffer = np.empty((0,2))      # current buffer with timestamp in seconds
        self.voltage_buffer = np.empty((0,2))      # voltage buffer with timestamp in seconds
        self.wfs_buffer = np.empty((0,2))      # voltage buffer with timestamp in seconds
        self.end_robot_buffer = np.empty((0,8))    # end robot pose (location with orientation) buffer with timestamp in seconds
        self.twist_robot_buffer = np.empty((0,7))    # end robot pose (location with orientation) buffer with timestamp in seconds
        self.travel_speed_buffer = np.empty((0,2))    # end robot pose (location with orientation) buffer with timestamp in seconds
        self.arc_on =  False
        self.flow_on =  False
        self.audio_on =  False

        self.deposition_data = np.empty((0,7))
        self.first_prediciton = True

        self.compute_max_min_by_data = False
        self.maximum_audiopress = 0.0
        self.minimum_audiopress = 0.0
        self.maximum_fftpower = 0.0
        self.minimum_fftpower = 0.0
        self.maximum_fftphase = 0.0
        self.minimum_fftphase = 0.0
        self.maximum_current = 0.0
        self.minimum_current = 0.0
        self.maximum_voltage = 0.0
        self.minimum_voltage = 0.0
        self.maximum_travel = 0.0
        self.minimum_travel = 0.0
        self.maximum_wfs = 0.0
        self.minimum_wfs = 0.0

        self.model_name = "defaut.h5"
        self.exp_name = "defautl_name"
        self.seq_count = 0
        self.resultVec = np.empty((0,3))
        self.beginPred = False

        # initialization ros node
        self.ros_init()

    # ros node initialization:
    def ros_init(self):
        # init node
        rospy.init_node('audio_tracking', anonymous=True)

        # ros parameters
        self.use_sim_time = rospy.get_param('/use_sim_time', default=False)

        self.folder_name = rospy.get_param('audio_tracking/folder_path')                               # database folder to trainning the Neural Network
        self.path_name = rospy.get_param('audio_tracking/path_path')                                   # configuration path to save the Neural Network coeficients
        self.exp_name = rospy.get_param('audio_tracking/exp_name')

        self.type = rospy.get_param('audio_tracking/type')                                             # work type: training or prediction

        self.window = rospy.get_param('audio_tracking/window')                                         # data window length in seconds (audio, current or voltage)
        self.training_percentage = rospy.get_param('audio_tracking/training_percentage')               # enabel cross validation
        self.cross_validation = rospy.get_param('audio_tracking/cross_validation')                     # enabel cross validation
        self.number_slipt = rospy.get_param('audio_tracking/number_slipt')                             # number of groups in cross validatation
        self.predict_rate = rospy.get_param('audio_tracking/predict_rate')                             # prediction rate for audio defect analysis
        self.model_name = rospy.get_param('audio_tracking/model_name')

        self.epochs = rospy.get_param('audio_tracking/epochs')                                         # Epochs
        self.neurons = rospy.get_param('audio_tracking/neurons')                                       # hidden neurons
        self.layers = rospy.get_param('audio_tracking/layers')                                         # layers
        self.batch_size = rospy.get_param('audio_tracking/batch_size')                                 # batch_size

        self.audio_topic = rospy.get_param('audio_tracking/audio_topic')                               # stamped audio topic
        self.audio_info_topic = rospy.get_param('audio_tracking/audio_info_topic')                     # audio info

        self.sample_fronius = rospy.get_param('audio_tracking/sample_fronius')                         # froniuns sample rate 
        self.froniuns_state_topic = rospy.get_param('audio_tracking/froniuns_state_topic')             # fronius state topic
        self.arc_state_topic = rospy.get_param('audio_tracking/arc_state_topic')                       # arc on topic
        self.audio_command_topic = rospy.get_param('audio_tracking/audio_command_topic')                       # arc on topic
        
        self.robot_end_position_topic = rospy.get_param('audio_tracking/robot_end_position_topic')     # robot end position
        self.robot_end_twist_topic = rospy.get_param('audio_tracking/robot_end_twist_topic')        # robot end position
        self.robot_end_travel_speed_topic = rospy.get_param('audio_tracking/robot_end_travel_speed_topic') # robot end position
        self.base_frame = rospy.get_param('audio_tracking/base_frame')
        self.deposition_frame = rospy.get_param('audio_tracking/deposition_frame')

        self.compute_max_min_by_data = rospy.get_param('audio_tracking/compute_max_min_by_data') 
        self.maximum_audiopress = rospy.get_param('audio_tracking/max_audio') 
        self.minimum_audiopress = rospy.get_param('audio_tracking/min_audio') 
        self.maximum_fftpower = rospy.get_param('audio_tracking/max_fftpower') 
        self.minimum_fftpower = rospy.get_param('audio_tracking/min_fftpower') 
        self.maximum_fftphase = rospy.get_param('audio_tracking/max_fftphase') 
        self.minimum_fftphase = rospy.get_param('audio_tracking/min_fftphase') 
        self.maximum_current = rospy.get_param('audio_tracking/max_current') 
        self.minimum_current = rospy.get_param('audio_tracking/min_current') 
        self.maximum_voltage = rospy.get_param('audio_tracking/max_voltage') 
        self.minimum_voltage = rospy.get_param('audio_tracking/min_voltage') 
        self.maximum_travel = rospy.get_param('audio_tracking/max_travel') 
        self.minimum_travel = rospy.get_param('audio_tracking/min_travel') 
        self.maximum_wfs = rospy.get_param('audio_tracking/max_wfs') 
        self.minimum_wfs = rospy.get_param('audio_tracking/min_wfs') 

        self.maximum_fftphase = self.maximum_fftphase*pi/180.0
        self.minimum_fftphase = self.minimum_fftphase*pi/180.0

        # subscribers;
        # if (self.use_sim_time):
        #     rospy.Subscriber('/clock', Clock, self.clock_callback)
        rospy.Subscriber(self.audio_topic, AudioDataStamped, self.audio_callback)
        rospy.Subscriber(self.audio_info_topic, AudioInfo,self.audioInfo_callback)
        rospy.Subscriber(self.froniuns_state_topic, Fronius500iState,self.fronius_callback)
        rospy.Subscriber(self.arc_state_topic, Bool,self.arc_callback)
        rospy.Subscriber(self.audio_command_topic, Bool,self.audio_command_callback)
        rospy.Subscriber(self.robot_end_position_topic, PoseStamped,self.robot_callback)
        rospy.Subscriber(self.robot_end_twist_topic, TwistStamped,self.twist_callback)
        rospy.Subscriber(self.robot_end_travel_speed_topic, Float64,self.travel_speed_callback)

        # publishers
        self.pub_res = rospy.Publisher('defect_found', audio_trackin_result, queue_size=10)

        # services

        # listenner
        self.listener = tf.TransformListener()

    # callbacks
    def clock_callback(self, msg):
        self.current_time = msg.clock

    def audio_callback(self, msg):
        if ((self.InfoRead and self.arc_on)):
        # if ((self.InfoRead)):
            # get time 
            time = msg.header.stamp.to_sec()

            # Determine the element size based on the data type
            element_size = np.dtype(np.int16).itemsize

            # Calculate the number of elements that fit within the buffer size
            num_elements = len(msg.audio.data) // element_size
            audio_data_aux = msg.audio.data[:num_elements * element_size]
            audio_data = np.frombuffer(np.array(audio_data_aux), dtype=np.int16)

            air_pressure = audio_data / 32767.0

            # if (self.InfoRead):
            for i in range(len(air_pressure)):
                # verify buffer length that is setted as 10 times windows of seconds.
                if(len(self.audio_buffer) > self.sample_rate*(self.window)+1):
                    aux = self.audio_buffer[1:len(self.audio_buffer),:]
                    self.audio_buffer = aux

                # increase the audio buffer    
                aux = [[time + i*(1/self.sample_rate), air_pressure[i]]]  # the audio data (int16) is divided by 32767.0 to converte to air pressure /32767.0
                self.audio_buffer = np.append(self.audio_buffer, aux, axis=0)
    
    def fronius_callback(self,msg):
        # get time 
        time = msg.header.stamp.to_sec()

        if (self.InfoRead and self.arc_on):
            # verify buffer length that is setted as 10 times windows of seconds.
            if(len(self.current_buffer) > self.sample_fronius*(self.window)+1):
                aux = self.current_buffer[1:len(self.current_buffer),:]
                self.current_buffer = aux
                aux = self.voltage_buffer[1:len(self.voltage_buffer),:]
                self.voltage_buffer = aux
                aux = self.wfs_buffer[1:len(self.wfs_buffer),:]
                self.wfs_buffer = aux
            
            # increase the buffer
            aux = [[time, msg.welding_current]] # change for kr10
            self.current_buffer = np.append(self.current_buffer, aux, axis=0)
            aux = [[time, msg.welding_voltage]] # change for k10
            self.voltage_buffer = np.append(self.voltage_buffer, aux, axis=0)
            aux = [[time, msg.wire_feed_speed]] # change for k10
            self.wfs_buffer = np.append(self.wfs_buffer, aux, axis=0)

    def robot_callback(self, msg):
        # get time 
        time = msg.header.stamp.to_sec()

        if (self.InfoRead and self.arc_on):
            # verify buffer length that is setted as 10 times windows of seconds.
            if(len(self.end_robot_buffer) > self.sample_fronius*(self.window)+1):
                aux = self.end_robot_buffer[1:len(self.end_robot_buffer),:]
                self.end_robot_buffer = aux
            
            # increase the buffer
            aux = [[time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]]
            self.end_robot_buffer = np.append(self.end_robot_buffer, aux, axis=0)

            # print("end_position (length) = ", len(self.end_robot_buffer))

    def twist_callback(self, msg):
        # get time 
        time = msg.header.stamp.to_sec()

        if (self.InfoRead and self.audio_on): # verify fronius frequency
            # verify buffer length that is setted as 10 times windows of seconds.
            if(len(self.twist_robot_buffer) > self.sample_fronius*(self.window)+1):
                aux = self.twist_robot_buffer[1:len(self.twist_robot_buffer),:]
                self.twist_robot_buffer = aux

            # increase the buffer
            aux = [[time, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]]
            self.twist_robot_buffer = np.append(self.twist_robot_buffer, aux, axis=0)

    def travel_speed_callback(self, msg):
        # get time 
        time = rospy.get_time()

        if (self.InfoRead and self.audio_on):
            # verify buffer length that is setted as 10 times windows of seconds.
            if(len(self.travel_speed_buffer) > self.sample_fronius*(self.window)+1):
                aux = self.travel_speed_buffer[1:len(self.travel_speed_buffer),:]
                self.travel_speed_buffer = aux

            # increase the buffer
            aux = [[time, msg.data]]
            self.travel_speed_buffer = np.append(self.travel_speed_buffer, aux, axis=0)

    def arc_callback(self,msg):
        # set arc_on or arc_off
        self.arc_on = msg.data

    def audio_command_callback(self,msg):
        # set audio_on or arc_off
        self.audio_on = msg.data

    def audioInfo_callback(self, msg):
        # audio info callback
        if (not(self.InfoRead)):
            self.channels = msg.channels
            self.sample_rate = msg.sample_rate
            self.sample_format = msg.sample_format
            self.bitrate = msg.bitrate
            self.coding_format = msg.coding_format

            self.InfoRead =  True

    # auxiliar functions
    def backspace(self, n):
        sys.stdout.write((b'\x08' * n).decode())

    def plot_audio(self, audio_data, frame_rate):
        # compute time vector
        time_vec = []

        # period
        delta_t = 1.0/float(frame_rate)
        time_t = 0.0
        data = []

        for i in range(len(audio_data)):
            if(len(audio_data.shape)>1):
                data = np.append(data, [audio_data[i][1]], axis=0)
            else:
                data = np.append(data, [audio_data[i]], axis=0)
            time_t += delta_t
            time_vec.append(time_t)

        # Plot the vector as a line
        plt.plot(time_vec,data)

        # Add a title and axis labels
        plt.title('Audio Plot')
        plt.xlabel('Time [s]')
        plt.ylabel('Air Pressure')

        plt.show()

    def read_audio_files(self, file_name):
        # read file
        with wave.open(file_name, 'rb') as file:
            # Get the audio file's parameters
            framerate = file.getframerate()
            nframes = file.getnframes()

            # Read the audio data
            audio_frames = file.readframes(nframes)

            # Convert the byte string to a numpy array of integers
            audio_data = np.frombuffer(audio_frames, dtype=np.int16)

            air_pressure = audio_data / 32767.0

            # self.plot_audio(air_pressure, framerate)

        return framerate, air_pressure

    def read_file_names(self, file_name):
        # Open the file and read the contents
        
        with open(file_name, 'r') as file:
            # Read the lines of the file
            lines = file.readlines()
            # Remove the first line (header)
            lines.pop(0)

        data = [line.strip().split(',') for line in lines]

        return data

    def audio_fft(self, audio_file):
        # Compute the DFT using numpy.fft.fft
        X = np.fft.fft(audio_file)

        # Compute the power spectrum (magnitude squared of the DFT)
        power_spectrum = np.abs(X)**2
        phase_spectrum = np.angle(X)

        plot_fft = False
        if(plot_fft):
            # compute time vector
            time_vec = []

            # period
            delta_t = 1.0/float(self.frame_rate)
            time_t = 0.0

            for i in range(len(audio_file)):
                time_t += delta_t
                time_vec.append(time_t)

            # Compute the frequency axis
            freqs = np.fft.fftfreq(len(audio_file), 1.0/float(self.frame_rate))

            # Plot the signal and its power spectrum
            fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, figsize=(10, 9))
            ax1.plot(time_vec, audio_file)
            ax1.set_xlabel('Time [s]')
            ax1.set_ylabel('Amplitude')
            ax2.plot(freqs, power_spectrum)
            ax2.set_xlabel('Frequency [Hz]')
            ax2.set_ylabel('Power')
            ax3.plot(freqs, phase_spectrum)
            ax3.set_xlabel('Frequency [Hz]')
            ax3.set_ylabel('Phase')
            plt.show()

        return power_spectrum, phase_spectrum

    def create_submodel(self, input_layer):
        # Create a submodel for a specific input
        model = Sequential()
        model.add(InputLayer(input_tensor=input_layer))
        model.add(Dense(64, activation='relu'))
        return model

    # Neural NetWork Functions
    def neural_network_thread(self):
        # Create input layers for each input
        input_audioPress = Input(shape=(len(self.inputs_audio_pressure[0]),), name='audio_pressure')
        input_audiofftpower = Input(shape=(len(self.inputs_audio_fftpower[0]),), name='audio_fftpower')
        input_audiofftphase = Input(shape=(len(self.inputs_audio_fftphase[0]),), name='audio_fftphase')
        input_current = Input(shape=(len(self.inputs_current_data[0]),), name='current_dense')
        input_voltage = Input(shape=(len(self.inputs_voltage_data[0]),), name='voltage_dense')
        input_travel_speed = Input(shape=(len(self.inputs_travel_speed_data[0]),), name='travel_speed_dense')
        input_wire_feed_speed = Input(shape=(len(self.inputs_wire_feed_speed_data[0]),), name='wire_feed_speed_dense')

        # Create separate models for each input
        model_audioPress = self.create_submodel(input_audioPress)
        model_audiofftpower = self.create_submodel(input_audiofftpower)
        model_audiofftphase = self.create_submodel(input_audiofftphase)
        model_current = self.create_submodel(input_current)
        model_voltage = self.create_submodel(input_voltage)
        model_travel_speed = self.create_submodel(input_travel_speed)
        model_wire_feed_speed = self.create_submodel(input_wire_feed_speed)

        # Concatenate the outputs of all models
        concatenated = Concatenate()([model_audioPress.output, model_audiofftpower.output, model_audiofftphase.output,
                                      model_current.output, model_voltage.output, model_travel_speed.output, model_wire_feed_speed.output])

        # Additional layers if needed
        for _ in range(self.layers - 1):
            concatenated = Dense(64, activation='relu')(concatenated)

        # Two output branches for binary classification - [0 to 5]
        output1 = Dense(6, activation='softmax', name='output1')(concatenated)
        output2 = Dense(6, activation='softmax', name='output2')(concatenated)

        # Create the final model
        model = Model(inputs=[input_audioPress, input_audiofftpower, input_audiofftphase, input_current,
                              input_voltage, input_travel_speed, input_wire_feed_speed],
                      outputs=[output1, output2])

        # Compile the model
        model.compile(optimizer='adam',
                      loss={'output1': 'sparse_categorical_crossentropy', 'output2': 'sparse_categorical_crossentropy'},
                      metrics={'output1': 'accuracy', 'output2': 'accuracy'})

        # Print model summary
        model.summary()
        # Define the file path where you want to save the model summary
        summary_file_path = self.path_name + 'trainning_history/model_summary.txt'
        # Write the model summary to a text file
        with open(summary_file_path, 'w') as f:
             with redirect_stdout(f):
                model.summary()

        # traning
        Xap_test = self.inputs_audio_pressure_test
        Xpw_test = self.inputs_audio_fftpower_test
        Xph_test = self.inputs_audio_fftphase_test
        Xc_test = self.inputs_current_data_test
        Xv_test = self.inputs_voltage_data_test
        Xt_test = self.inputs_travel_speed_data_test
        Xw_test = self.inputs_wire_feed_speed_data_test
        y_test = self.output_test 

        first_model =  True

        anomaly_percentage = []
        hits_anomaly_percentage = []
        hits_anomaly_type_results = []
        hits_anomaly_classification_results = []

        if (self.cross_validation):
            hits_percentage = np.empty((0,))

            # Perform 10-fold cross-validation
            # kf = KFold(n_splits=self.number_slipt)
            # Convert multi-output labels to a single label
            combined_labels = [f"{label1}-{label2}" for label1, label2 in zip(self.output_training[:, 0], self.output_training[:, 1])]

            # Encode the combined labels
            label_encoder = LabelEncoder()
            encoded_labels = label_encoder.fit_transform(combined_labels)

            kf = StratifiedKFold(n_splits=self.number_slipt)
            data_indice = 1

            for train_index, test_index in kf.split(self.inputs_audio_pressure_training, encoded_labels):
                Xap_train, Xap_val = self.inputs_audio_pressure_training[train_index], self.inputs_audio_pressure_training[test_index]
                Xpw_train, Xpw_val = self.inputs_audio_fftpower_training[train_index], self.inputs_audio_fftpower_training[test_index]
                Xph_train, Xph_val = self.inputs_audio_fftphase_training[train_index], self.inputs_audio_fftphase_training[test_index]
                Xc_train, Xc_val = self.inputs_current_data_training[train_index], self.inputs_current_data_training[test_index]
                Xv_train, Xv_val = self.inputs_voltage_data_training[train_index], self.inputs_voltage_data_training[test_index]
                Xt_train, Xt_val = self.inputs_travel_speed_data_training[train_index], self.inputs_travel_speed_data_training[test_index]
                Xw_train, Xw_val = self.inputs_wire_feed_speed_data_training[train_index], self.inputs_wire_feed_speed_data_training[test_index]
                y_train, y_val = self.output_training[train_index], self.output_training[test_index]

                print("shape input = ", self.inputs_audio_pressure_training.shape, "\n")
                print("shape input2 = ", self.inputs_current_data_training.shape, "\n")
                print("shape X = ", Xap_train.shape, "\n")
                print("shape X2 = ", Xc_train.shape, "\n")
                print("shape Y = ", y_train.shape, "\n")

                # Train the neural network using the training data
                history = model.fit({'input_1': Xap_train, 'input_2': Xpw_train, 'input_3': Xph_train, 'input_4': Xc_train, 'input_5': Xv_train, 'input_6': Xt_train, 'input_7': Xw_train},
                          {'output1': y_train[:, 0], 'output2': y_train[:, 1]},
                          epochs=self.epochs, batch_size=self.batch_size,
                          validation_data=({'input_1': Xap_val, 'input_2': Xpw_val, 'input_3': Xph_val, 'input_4': Xc_val,
                                            'input_5': Xv_val, 'input_6': Xt_val, 'input_7': Xw_val},
                                           {'output1': y_val[:, 0], 'output2': y_val[:, 1]}))
                
                # saving results
                model.save(self.path_name+'trainning_history/audio_tracking_model_' + str(data_indice) +'.h5')

                # Save the training history to a CSV file
                history_df = pd.DataFrame(history.history)
                history_csv_file = self.path_name + 'trainning_history/training_history_' + str(data_indice) + '.csv'
                history_df.to_csv(history_csv_file, index=False)    

                # Use the trained neural network to make predictions on new data
                predictions = model.predict({
                    'input_1': Xap_val,
                    'input_2': Xpw_val,
                    'input_3': Xph_val,
                    'input_4': Xc_val,
                    'input_5': Xv_val,
                    'input_6': Xt_val,
                    'input_7': Xw_val
                })

                # You can convert them to a pandas DataFrame
                predictions_df = pd.DataFrame(predictions[0])
                # Define the file path where you want to save the results
                csv_file_path = self.path_name + 'trainning_history/predictions_out1_' + str(data_indice)+ '.csv'
                # Save to CSV
                predictions_df.to_csv(csv_file_path, index=False)

                predictions_df = pd.DataFrame(predictions[1])
                # Define the file path where you want to save the results
                csv_file_path = self.path_name + 'trainning_history/predictions_out2_' + str(data_indice)+ '.csv'
                # Save to CSV
                predictions_df.to_csv(csv_file_path, index=False)

                y_test_df = pd.DataFrame(y_val[:,0])
                # Define the file path where you want to save the results
                csv_file_path = self.path_name + 'trainning_history/y_out1_' + str(data_indice)+ '.csv'
                # Save to CSV
                y_test_df.to_csv(csv_file_path, index=False)

                y_test_df = pd.DataFrame(y_val[:,1])
                # Define the file path where you want to save the results
                csv_file_path = self.path_name + 'trainning_history/y_out2_' + str(data_indice)+ '.csv'
                # Save to CSV
                y_test_df.to_csv(csv_file_path, index=False)

                data_indice = data_indice + 1

                output1_predictions = np.argmax(predictions[0], axis=1)
                output2_predictions = np.argmax(predictions[1], axis=1)

                # validation  - Stop HERE
                anomaly_presence = 0
                anomaly_results = 0
                anomaly_type_results = 0
                anomaly_classification_results = 0

                for i in range(len(predictions[0])):
                    # comparison - only if anomaly is identified
                    if (y_val[i,0] > 0):
                        anomaly_presence += 1
                    if (y_val[i,0] > 0 and output1_predictions[i] > 0):
                        anomaly_results += 1
                    elif (y_val[i,0] == output1_predictions[i]):
                        anomaly_results += 1

                    # comparison - only if the anomaly type is identified
                    if (y_val[i,0] == output1_predictions[i]):
                        anomaly_type_results += 1
                        # complete anomaly is identified
                        if (y_val[i,1] == output2_predictions[i]):
                            anomaly_classification_results += 1   
                

                # save results
                anomaly_percentage = np.append(anomaly_percentage, anomaly_presence/len(predictions[0]))
                hits_anomaly_percentage = np.append(hits_anomaly_percentage, anomaly_results/len(predictions[0]))
                hits_anomaly_type_results = np.append(hits_anomaly_type_results, anomaly_type_results/len(predictions[0]))
                hits_anomaly_classification_results = np.append(hits_anomaly_classification_results, anomaly_classification_results/len(predictions[0]))

            # cross validation result
            # mean_anomaly_hits = np.sum(hits_anomaly_percentage)/float(len(hits_anomaly_percentage))
            # mean_anomaly_type_results = np.sum(hits_anomaly_type_results)/float(len(hits_anomaly_type_results))
            # mean_anomaly_classification_results = np.sum(hits_anomaly_classification_results)/float(len(hits_anomaly_classification_results))

            mean_anomaly_hits = hits_anomaly_percentage[-1]
            mean_anomaly_type_results = hits_anomaly_type_results[-1]
            mean_anomaly_classification_results = hits_anomaly_classification_results[-1]

            # Write the list to a CSV file
            csv_file_path = self.path_name + 'trainning_history/anomaly_percentage.csv'
            with open(csv_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(anomaly_percentage)

            csv_file_path = self.path_name + 'trainning_history/hits_anomaly_percentage.csv'
            with open(csv_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(hits_anomaly_percentage)

            csv_file_path = self.path_name + 'trainning_history/hits_anomaly_type_results.csv'
            with open(csv_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(hits_anomaly_type_results)

            csv_file_path = self.path_name + 'trainning_history/hits_anomaly_classification_results.csv'
            with open(csv_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(hits_anomaly_classification_results)

            csv_file_path = self.path_name + 'trainning_history/hits_results.txt'
            result_string = "Trainning Results: \n" + "mean_anomaly_hits = " + str(mean_anomaly_hits) + '\n' + "mean_anomaly_type_results = " + str(mean_anomaly_type_results) + '\n' + "mean_anomaly_classification_results = " + str(mean_anomaly_classification_results)            
            
            with open(csv_file_path, 'w') as f:
                f.write(result_string)

            print("Percentage of Anomaly Hits =", mean_anomaly_hits*100.0, "%" )
            print("Percentage of Anomaly type Hits =", mean_anomaly_type_results*100.0, "%" )
            print("Percentage of Anomaly Classification Hits =", mean_anomaly_classification_results*100.0, "%" )

        else:
            Xap_train = self.inputs_audio_pressure_training
            Xpw_train = self.inputs_audio_fftpower_training
            Xph_train = self.inputs_audio_fftphase_training
            Xc_train = self.inputs_current_data_training
            Xv_train = self.inputs_voltage_data_training
            Xt_train = self.inputs_travel_speed_data_training
            Xw_train = self.inputs_wire_feed_speed_data_training
            y_train = self.output_training

            # Train the neural network using the training data
            history = model.fit({'audio_pressure': Xap_train, 'audio_fftpower': Xpw_train, 'audio_fftphase': Xph_train, 'current_dense': Xc_train,
                        'voltage_dense': Xv_train, 'travel_speed_dense': Xt_train, 'wire_feed_speed_dense': Xw_train},
                        {'output1': y_train[:,0], 'output2': y_train[:,1]}, 
                        epochs=self.epochs, batch_size=self.batch_size, 
                        validation_data=({'audio_pressure': Xap_val, 'audio_fftpower': Xpw_val, 'audio_fftphase': Xph_val, 'current_dense': Xc_val,
                                            'voltage_dense': Xv_val, 'travel_speed_dense': Xt_val, 'wire_feed_speed_dense': Xw_val}))

            # Use the trained neural network to make predictions on new data
            predictions = model.predict({
                'audio_pressure': Xap_val,
                'audio_fftpower': Xpw_val,
                'audio_fftphase': Xph_val,
                'current_dense': Xc_val,
                'voltage_dense': Xv_val,
                'travel_speed_dense': Xt_val,
                'wire_feed_speed_dense': Xw_val
            })

            # Extract predictions for each output
            output1_predictions = np.argmax(predictions['output1'], axis=1)
            output2_predictions = np.argmax(predictions['output2'], axis=1)

            # validation  - Stop HERE
            anomaly_results = 0
            anomaly_type_results = 0
            anomaly_classification_results = 0

            for i in range(len(predictions)):
                # comparison - only if anomaly is identified
                if (y_val[i,0] > 0 and output1_predictions[i] > 0):
                    anomaly_results += 1
                elif (y_val[i,0] == output1_predictions[i]):
                    anomaly_results += 1

                # comparison - only if the anomaly type is identified
                if (y_test[i,0] == output1_predictions[i]):
                    anomaly_type_results += 1
                    # complete anomaly is identified
                    if (y_val[i,1] == output2_predictions[i]):
                        anomaly_classification_results += 1                    

            # save results
            hits_anomaly_percentage = anomaly_results/len(predictions)
            hits_anomaly_percentage = anomaly_type_results/len(predictions)
            hits_anomaly_classification_results = anomaly_classification_results/len(predictions)

            print("Percentage of Anomaly Hits =", hits_anomaly_percentage*100.0, "%" )
            print("Percentage of Anomaly type Hits =", mean_anomaly_type_results*100.0, "%" )
            print("Percentage of Anomaly Classification Hits =", hits_anomaly_classification_results*100.0, "%" )

        # ====== creating a model with all of data ===
        Xap_train = self.inputs_audio_pressure_training
        Xpw_train = self.inputs_audio_fftpower_training
        Xph_train = self.inputs_audio_fftphase_training
        Xc_train = self.inputs_current_data_training
        Xv_train = self.inputs_voltage_data_training
        Xt_train = self.inputs_travel_speed_data_training
        Xw_train = self.inputs_wire_feed_speed_data_training
        y_train = self.output_training

        Xap_test = self.inputs_audio_pressure_test
        Xpw_test = self.inputs_audio_fftpower_test
        Xph_test = self.inputs_audio_fftphase_test 
        Xc_test  = self.inputs_current_data_test
        Xv_test  = self.inputs_voltage_data_test
        Xt_test  = self.inputs_travel_speed_data_test
        Xw_test  = self.inputs_wire_feed_speed_data_test
        y_test   = self.output_test

        history = model.fit({'input_1': Xap_train, 'input_2': Xpw_train, 'input_3': Xph_train, 'input_4': Xc_train, 'input_5': Xv_train, 'input_6': Xt_train, 'input_7': Xw_train},
                    {'output1': y_train[:, 0], 'output2': y_train[:, 1]},
                    epochs=self.epochs, batch_size=self.batch_size,
                    validation_data=({'input_1': Xap_test, 'input_2': Xpw_test, 'input_3': Xph_test, 'input_4': Xc_test,
                                    'input_5': Xv_test, 'input_6': Xt_test, 'input_7': Xw_test},
                                    {'output1': y_test[:, 0], 'output2': y_test[:, 1]}))
        # save model
        model.save(self.path_name+'trainning_history/audio_tracking_model_complete.h5')

        # Save the training history to a CSV file
        history_df = pd.DataFrame(history.history)
        history_csv_file = self.path_name + 'trainning_history/training_history_complete.csv'
        history_df.to_csv(history_csv_file, index=False)    

        # Use the trained neural network to make predictions on new data
        predictions = model.predict({
            'input_1': Xap_train,
            'input_2': Xpw_train,
            'input_3': Xph_train,
            'input_4': Xc_train,
            'input_5': Xv_train,
            'input_6': Xt_train,
            'input_7': Xw_train
        })

        # You can convert them to a pandas DataFrame
        predictions_df = pd.DataFrame(predictions[0])
        # Define the file path where you want to save the results
        csv_file_path = self.path_name + 'trainning_history/predictions_out1_complete.csv'
        # Save to CSV
        predictions_df.to_csv(csv_file_path, index=False)

        predictions_df = pd.DataFrame(predictions[1])
        # Define the file path where you want to save the results
        csv_file_path = self.path_name + 'trainning_history/predictions_out2_complete.csv'
        # Save to CSV
        predictions_df.to_csv(csv_file_path, index=False)

        y_test_df = pd.DataFrame(y_val[:,0])
        # Define the file path where you want to save the results
        csv_file_path = self.path_name + 'trainning_history/y_out1_complete.csv'
        # Save to CSV
        y_test_df.to_csv(csv_file_path, index=False)

        y_test_df = pd.DataFrame(y_val[:,1])
        # Define the file path where you want to save the results
        csv_file_path = self.path_name + 'trainning_history/y_out2_complete.csv'
        # Save to CSV
        y_test_df.to_csv(csv_file_path, index=False)

        data_indice = data_indice + 1

        output1_predictions = np.argmax(predictions[0], axis=1)
        output2_predictions = np.argmax(predictions[1], axis=1)

        # validation  - Stop HERE
        anomaly_presence = 0
        anomaly_results = 0
        anomaly_type_results = 0
        anomaly_classification_results = 0

        for i in range(len(predictions[0])):
            # comparison - only if anomaly is identified
            if (y_val[i,0] > 0):
                anomaly_presence += 1
            if (y_val[i,0] > 0 and output1_predictions[i] > 0):
                anomaly_results += 1
            elif (y_val[i,0] == output1_predictions[i]):
                anomaly_results += 1

            # comparison - only if the anomaly type is identified
            if (y_val[i,0] == output1_predictions[i]):
                anomaly_type_results += 1
                # complete anomaly is identified
                if (y_val[i,1] == output2_predictions[i]):
                    anomaly_classification_results += 1   
        

        # save results
        anomaly_percentage = np.append(anomaly_percentage, anomaly_presence/len(predictions[0]))
        hits_anomaly_percentage = np.append(hits_anomaly_percentage, anomaly_results/len(predictions[0]))
        hits_anomaly_type_results = np.append(hits_anomaly_type_results, anomaly_type_results/len(predictions[0]))
        hits_anomaly_classification_results = np.append(hits_anomaly_classification_results, anomaly_classification_results/len(predictions[0]))

        # Write the list to a CSV file
        csv_file_path = self.path_name + 'trainning_history/anomaly_percentage.csv'
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(anomaly_percentage)

        csv_file_path = self.path_name + 'trainning_history/hits_anomaly_percentage.csv'
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(hits_anomaly_percentage)

        csv_file_path = self.path_name + 'trainning_history/hits_anomaly_type_results.csv'
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(hits_anomaly_type_results)

        csv_file_path = self.path_name + 'trainning_history/hits_anomaly_classification_results.csv'
        with open(csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(hits_anomaly_classification_results)

        csv_file_path = self.path_name + 'trainning_history/hits_results_complete.txt'
        result_string = "Trainning Results: \n" + "mean_anomaly_hits = " + str(mean_anomaly_hits) + '\n' + "mean_anomaly_type_results = " + str(mean_anomaly_type_results) + '\n' + "mean_anomaly_classification_results = " + str(mean_anomaly_classification_results)            
        
        with open(csv_file_path, 'w') as f:
            f.write(result_string)

        print("Percentage of Anomaly Hits =", mean_anomaly_hits*100.0, "%" )
        print("Percentage of Anomaly type Hits =", mean_anomaly_type_results*100.0, "%" )
        print("Percentage of Anomaly Classification Hits =", mean_anomaly_classification_results*100.0, "%" )
        
        # saving results
        model.save(self.path_name+'audio_tracking_model_complete.h5')

        # finish
        self.finish_training = True

    def load_predict_model(self):
        # load model
        self.predict_model = load_model(self.path_name+self.model_name)

        # load normalization variables
        if self.compute_max_min_by_data:
            with open(self.path_name+'maximum_audiopress.pkl', 'rb') as file:
                self.maximum_audiopress = pickle.load(file)

            with open(self.path_name+'minimum_audiopress.pkl', 'rb') as file:
                self.minimum_audiopress = pickle.load(file)
            
            with open(self.path_name+'maximum_fftpower.pkl', 'rb') as file:
                self.maximum_fftpower = pickle.load(file)
            
            with open(self.path_name+'minimum_fftpower.pkl', 'rb') as file:
                self.minimum_fftpower = pickle.load(file)
            
            with open(self.path_name+'maximum_fftphase.pkl', 'rb') as file:
                self.maximum_fftphase = pickle.load(file)
            
            with open(self.path_name+'minimum_fftphase.pkl', 'rb') as file:
                self.minimum_fftphase = pickle.load(file)

            with open(self.path_name+'maximum_current.pkl', 'rb') as file:
                self.maximum_current = pickle.load(file)

            with open(self.path_name+'minimum_crrent.pkl', 'rb') as file:
                self.minimum_crrent = pickle.load(file)
            
            with open(self.path_name+'maximum_voltage.pkl', 'rb') as file:
                self.maximum_voltage = pickle.load(file)
            
            with open(self.path_name+'minimum_voltage.pkl', 'rb') as file:
                self.minimum_voltage = pickle.load(file)
            
            with open(self.path_name+'maximum_travel.pkl', 'rb') as file:
                self.maximum_travel = pickle.load(file)
            
            with open(self.path_name+'minimum_travel.pkl', 'rb') as file:
                self.minimum_travel = pickle.load(file)

            with open(self.path_name+'maximum_wfs.pkl', 'rb') as file:
                self.maximum_wfs = pickle.load(file)
            
            with open(self.path_name+'minimum_wfs.pkl', 'rb') as file:
                self.minimum_wfs = pickle.load(file)

    def input_data_construction(self):
        # copy the buffer to other memory alocation the avoid that the data change when the input data where constructed
        audio_buff = copy.deepcopy(self.audio_buffer)
        current_buff = copy.deepcopy(self.current_buffer)
        voltage_buff = copy.deepcopy(self.voltage_buffer)
        travel_buff = copy.deepcopy(self.travel_speed_buffer)
        wfs_buff = copy.deepcopy(self.wfs_buffer)
        robotPos_buff = copy.deepcopy(self.end_robot_buffer)

        # compute FFT
        # time_data = []
        audio_data = []
        current_data = []
        voltage_data = []
        travel_data = []
        wfs_data = []
        robot_data = []

        # for i in range(int(self.sample_rate*(self.window))):
        #     # audio data
        #     time_data = np.append(time_data, [self.audio_buffer[i][0]], axis=0)
        #     audio_data = np.append(audio_data, [self.audio_buffer[i][1]], axis=0)
        #     # current and voltage data
        #     if (i < int(self.sample_fronius*(self.window))+1):
        #         current_data = np.append(current_data, [self.current_buffer[i][1]],axis=0)
        #         voltage_data = np.append(voltage_data, [self.voltage_buffer[i][1]],axis=0)
        #         travel_data = np.append(travel_data, [self.travel_speed_buffer[i][1]],axis=0)
        #         wfs_data = np.append(wfs_data, [self.wfs_buffer[i][1]],axis=0)

        for i in range(int(self.sample_rate*(self.window))):
            # audio data
            # time_data = np.append(time_data, [audio_buff[i][0]], axis=0)
            audio_data = np.append(audio_data, [audio_buff[i][1]], axis=0)
            # current and voltage data
            if (i < int(self.sample_fronius*(self.window))+1):
                current_data = np.append(current_data, [current_buff[i][1]],axis=0)
                voltage_data = np.append(voltage_data, [voltage_buff[i][1]],axis=0)
                travel_data = np.append(travel_data, [travel_buff[i][1]],axis=0)
                wfs_data = np.append(wfs_data, [wfs_buff[i][1]],axis=0)
                robot_data = np.append(robot_data, [robotPos_buff[i][0]],axis=0)

        # fft
        fft_power, fft_phase = self.audio_fft(audio_data)

        # self.plot_audio(audio_data, self.sample_rate)

        # ==========  normalization  ===========
        # air pressure- change here
        audio_vector = np.array(audio_data)
        audio_vector = (np.maximum(np.minimum(audio_vector, self.maximum_audiopress), self.minimum_audiopress) - self.minimum_audiopress)/(self.maximum_audiopress - self.minimum_audiopress)
        # power spectrum
        power_vector = np.array(fft_power)
        power_vector = (np.maximum(np.minimum(power_vector, self.maximum_fftpower), self.minimum_fftpower) - self.minimum_fftpower)/(self.maximum_fftpower - self.minimum_fftpower)
        # phase spectrum
        phase_vector = np.array(fft_phase)
        phase_vector = (np.maximum(np.minimum(phase_vector, self.maximum_fftphase), self.minimum_fftphase) - self.minimum_fftphase)/(self.maximum_fftphase - self.minimum_fftphase)
        
        # current 
        current_vector = np.array(current_data)
        current_vector = (np.maximum(np.minimum(current_vector, self.maximum_current), self.minimum_current) - self.minimum_current)/(self.maximum_current - self.minimum_current)
        # voltage 
        voltage_vector = np.array(voltage_data)
        voltage_vector = (np.maximum(np.minimum(voltage_vector, self.maximum_voltage), self.minimum_voltage) - self.minimum_voltage)/(self.maximum_voltage - self.minimum_voltage)
        # travel speed 
        travel_vector = np.array(travel_data)
        travel_vector = (np.maximum(np.minimum(travel_vector, self.maximum_travel), self.minimum_travel) - self.minimum_travel)/(self.maximum_travel - self.minimum_travel)
        # wire feed speed 
        wfs_vector = np.array(wfs_data)
        wfs_vector = (np.maximum(np.minimum(wfs_vector, self.maximum_wfs), self.minimum_wfs) - self.minimum_wfs)/(self.maximum_wfs - self.minimum_wfs)
        # ==========================================


        # create a vector just to concatenate
        robot_data_out = np.empty((0, len(robot_data)))
        audio_data_out = np.empty((0, len(audio_vector)))
        fft_power_out = np.empty((0, len(fft_power)))
        fft_phase_out = np.empty((0, len(fft_phase)))
        current_data_out = np.empty((0, len(current_vector)))
        voltage_data_out = np.empty((0, len(voltage_vector)))
        travel_data_out = np.empty((0, len(travel_vector)))
        wfs_data_out = np.empty((0, len(wfs_vector)))

        # stacking 
        robot_data_out = np.vstack((robot_data_out, robot_data))
        audio_data_out = np.vstack((audio_data_out, audio_vector))
        fft_power_out = np.vstack((fft_power_out, fft_power))
        fft_phase_out = np.vstack((fft_phase_out, fft_phase))
        current_data_out = np.vstack((current_data_out, current_vector))
        voltage_data_out = np.vstack((voltage_data_out, voltage_vector))
        travel_data_out = np.vstack((travel_data_out, travel_vector))
        wfs_data_out = np.vstack((wfs_data_out, wfs_vector))
        
        return robot_data_out, audio_data_out, fft_power_out, fft_phase_out, current_data_out, voltage_data_out, travel_data_out, wfs_data_out
        # return audio_data_out, fft_power_out, fft_phase_out, current_data_out, voltage_data_out, travel_data_out, wfs_data_out

    def predition_defect(self):
        # generate input data
        if((len(self.audio_buffer) >= self.sample_rate*(self.window)) and len(self.current_buffer) >= self.sample_fronius*(self.window) and self.arc_on):
            self.first_prediciton = False

            robot_vector, audio_vector, power_vector, phase_vector, current_vector, voltage_vector, travel_vector, wfs_vector = [], [], [], [], [], [], [], []
            robot_vector, audio_vector, power_vector, phase_vector, current_vector, voltage_vector, travel_vector, wfs_vector = self.input_data_construction()
       
            # prediction
            predictions = self.predict_model.predict({
                    'input_1': audio_vector,
                    'input_2': power_vector,
                    'input_3': phase_vector,
                    'input_4': current_vector,
                    'input_5': voltage_vector,
                    'input_6': travel_vector,
                    'input_7': wfs_vector
                })

            # Extract predictions for each output
            output1_predictions = np.argmax(predictions[0], axis=1)
            output2_predictions = np.argmax(predictions[1], axis=1)

            # print("prediction = ", output1_predictions[0], output2_predictions[0],",\n")

            # publishing the result
            bool_msg = audio_trackin_result()
            bool_msg.header.stamp = rospy.Time.now()
            bool_msg.header.seq = self.seq_count
            if (output1_predictions >= 1):
                bool_msg.data = True  # Set the bool value
                # bool_msg.result = str(output1_predictions[0]) + str(output2_predictions[0])
            else:
                bool_msg.data = False  # Set the bool value
            
            bool_msg.result = str(output1_predictions[0]) + str(output2_predictions[0])
            self.pub_res.publish(bool_msg)

            self.seq_count += 1

            # results = [[rospy.Time.now().to_sec(), output1_predictions[0], output2_predictions[0]]]
            print("robot to print = ",robot_vector[-1][-1], ", size = ",robot_vector.shape)
            results = [[robot_vector[-1][-1], output1_predictions[0], output2_predictions[0]]]
            self.resultVec = np.append(self.resultVec, np.array(results), axis=0)

            self.first_prediciton = True
            self.beginPred = True

        if self.beginPred and not(self.arc_on):
            # save results
            txt_file_path = self.path_name + 'trainning_history/prediction_results' + self.exp_name + '.txt'
            with open(txt_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.resultVec)

            with open(txt_file_path, 'w', newline='') as f:
                writer = csv.writer(f)                
                # Loop through each row in self.resultVec
                for row in self.resultVec:
                    # Format each element with .9f precision
                    formatted_row = ['{:.10f}'.format(element) for element in row]
                    
                    # Write the formatted row to the file
                    writer.writerow(formatted_row)


    def load_audio_database(self):
        # load database
        database_matrix = self.read_file_names(self.folder_name + 'audio_database.txt')

        # read audios and output files
        first_data = True
        for i in range(len(database_matrix)):
            # skip first data
            if i == 0:
                continue

            # read audio database
            framerate, audio_data = self.read_audio_files(self.folder_name+database_matrix[i][0]+'.wav')

            # append to vector database
            if (first_data):
                # airs_pressure
                self.inputs_audio_pressure = np.empty((0, len(audio_data)))
                # power_spectrum of fft
                self.inputs_audio_fftpower = np.empty((0, len(audio_data)))
                # phase sprectrum of fft
                self.inputs_audio_fftphase = np.empty((0, len(audio_data)))
                # output data
                self.output_data = np.empty((0, 2))
                # audio parameters
                self.frame_rate = framerate
                self.audio_dim = len(audio_data)
                # bool
                first_data = False

            # compute FFT
            fft_power, fft_phase = self.audio_fft(audio_data)
        
            # saving
            self.inputs_audio_pressure = np.vstack((self.inputs_audio_pressure, np.array(audio_data)))
            self.inputs_audio_fftpower = np.vstack((self.inputs_audio_fftpower, np.array(fft_power)))
            self.inputs_audio_fftphase = np.vstack((self.inputs_audio_fftphase, np.array(fft_phase)))
            self.output_data = np.vstack((self.output_data, np.array(database_matrix[i][1:], dtype=np.float32)))

        # normalization
        if self.compute_max_min_by_data:
            self.maximum_audiopress = np.amax(self.inputs_audio_pressure)
            self.minimum_audiopress = np.amin(self.inputs_audio_pressure)
        self.inputs_audio_pressure  = (np.maximum(np.minimum(self.inputs_audio_pressure, self.maximum_audiopress), self.minimum_audiopress) - self.minimum_audiopress)/(self.maximum_audiopress - self.minimum_audiopress)
        
        if self.compute_max_min_by_data:
            self.maximum_fftpower = np.amax(self.inputs_audio_fftpower)
            self.minimum_fftpower = np.amin(self.inputs_audio_fftpower)
        self.inputs_audio_fftpower = (np.maximum(np.minimum(self.inputs_audio_fftpower, self.maximum_fftpower), self.minimum_fftpower) - self.minimum_fftpower)/(self.maximum_fftpower - self.minimum_fftpower)
        
        if self.compute_max_min_by_data:
            self.maximum_fftphase = np.amax(self.inputs_audio_fftphase)
            self.minimum_fftphase = np.amin(self.inputs_audio_fftphase)
        self.inputs_audio_fftphase = (np.maximum(np.minimum(self.inputs_audio_fftphase, self.maximum_fftphase), self.minimum_fftphase) - self.minimum_fftphase)/(self.maximum_fftphase - self.minimum_fftphase)
        
        # save the minimum and maximum
        if self.compute_max_min_by_data:
            with open(self.path_name+'maximum_audiopress.pkl', 'wb') as file:
                pickle.dump(self.maximum_audiopress, file)

            with open(self.path_name+'minimum_audiopress.pkl', 'wb') as file:
                pickle.dump(self.minimum_audiopress, file)
            
            with open(self.path_name+'maximum_fftpower.pkl', 'wb') as file:
                pickle.dump(self.maximum_fftpower, file)
            
            with open(self.path_name+'minimum_fftpower.pkl', 'wb') as file:
                pickle.dump(self.minimum_fftpower, file)
            
            with open(self.path_name+'maximum_fftphase.pkl', 'wb') as file:
                pickle.dump(self.maximum_fftphase, file)
            
            with open(self.path_name+'minimum_fftphase.pkl', 'wb') as file:
                pickle.dump(self.minimum_fftphase, file)

    def load_current_database(self):
        # load
        with open(self.folder_name + 'current.txt', 'r') as file:
            # Read all lines from the file
            lines = file.readlines()

            # Process each line
            first = True
            for line in lines:
                # Split the line into a list of values using commas as separators
                values = line.strip().split(',')

                if first:
                    first = False
                    self.inputs_current_data = np.empty((0, len(values)-2))
                    continue

                self.inputs_current_data = np.vstack((self.inputs_current_data, np.array(values[0:-2], dtype=np.float32)))

            print("length_current = ", len(self.inputs_current_data))

            # normalization
            if self.compute_max_min_by_data:
                self.maximum_current = np.amax(self.inputs_current_data)
                self.minimum_current = np.amin(self.inputs_current_data)
            self.inputs_current_data = (np.maximum(np.minimum(self.inputs_current_data, self.maximum_current), self.minimum_current) - self.minimum_current)/(self.maximum_current - self.minimum_current)

            # save the minimum and maximum
            if self.compute_max_min_by_data:
                with open(self.path_name+'maximum_current.pkl', 'wb') as file:
                    pickle.dump(self.maximum_current, file)

                with open(self.path_name+'maximum_current.pkl', 'wb') as file:
                    pickle.dump(self.minimum_current, file)

    def load_voltage_database(self):
        # load
        with open(self.folder_name + 'voltage.txt', 'r') as file:
            # Read all lines from the file
            lines = file.readlines()

            # Process each line
            first = True
            for line in lines:
                # Split the line into a list of values using commas as separators
                values = line.strip().split(',')

                if first:
                    first = False
                    self.inputs_voltage_data = np.empty((0, len(values)-2))
                    continue


                self.inputs_voltage_data = np.vstack((self.inputs_voltage_data, np.array(values[0:-2], dtype=np.float32)))

            print("length_voltage = ", len(self.inputs_voltage_data))

            # normalization
            if self.compute_max_min_by_data:
                self.maximum_voltage = np.amax(self.inputs_voltage_data)
                self.minimum_voltage = np.amin(self.inputs_voltage_data)
            self.inputs_voltage_data = (np.maximum(np.minimum(self.inputs_voltage_data, self.maximum_voltage), self.minimum_voltage) - self.minimum_voltage)/(self.maximum_voltage - self.minimum_voltage)
        
            # save the minimum and maximum
            if self.compute_max_min_by_data:
                with open(self.path_name+'maximum_voltage.pkl', 'wb') as file:
                    pickle.dump(self.maximum_voltage, file)

                with open(self.path_name+'minimum_voltage.pkl', 'wb') as file:
                    pickle.dump(self.minimum_voltage, file)
        
    def load_travel_speed_database(self):
        # load
        with open(self.folder_name + 'travel_speed.txt', 'r') as file:
            # Read all lines from the file
            lines = file.readlines()

            # Process each line
            first = True
            for line in lines:
                # Split the line into a list of values using commas as separators
                values = line.strip().split(',')

                if first:
                    first = False
                    self.inputs_travel_speed_data = np.empty((0, len(values)-2))
                    continue


                self.inputs_travel_speed_data = np.vstack((self.inputs_travel_speed_data, np.array(values[0:-2], dtype=np.float32)))

            print("length_travel_speed = ", len(self.inputs_travel_speed_data))

            # normalization
            if self.compute_max_min_by_data:
                self.maximum_travel = np.amax(self.inputs_travel_speed_data)
                self.minimum_travel = np.amin(self.inputs_travel_speed_data)
            self.inputs_travel_speed_data = (np.maximum(np.minimum(self.inputs_travel_speed_data, self.maximum_travel), self.minimum_travel) - self.minimum_travel)/(self.maximum_travel - self.minimum_travel)
        
            # save the minimum and maximum
            if self.compute_max_min_by_data:
                with open(self.path_name+'maximum_travel.pkl', 'wb') as file:
                    pickle.dump(self.maximum_travel, file)

                with open(self.path_name+'minimum_travel.pkl', 'wb') as file:
                    pickle.dump(self.minimum_travel, file)

    def load_wire_feed_speed_database(self):
        # load
        with open(self.folder_name + 'wire_feed_speed.txt', 'r') as file:
            # Read all lines from the file
            lines = file.readlines()

            # Process each line
            first = True
            for line in lines:
                # Split the line into a list of values using commas as separators
                values = line.strip().split(',')

                if first:
                    first = False
                    self.inputs_wire_feed_speed_data = np.empty((0, len(values)-2))
                    continue


                self.inputs_wire_feed_speed_data = np.vstack((self.inputs_wire_feed_speed_data, np.array(values[0:-2], dtype=np.float32)))

            print("length_wire_feed_speed = ", len(self.inputs_wire_feed_speed_data))

            # normalization
            if self.compute_max_min_by_data:
                self.maximum_wfs = np.amax(self.inputs_wire_feed_speed_data)
                self.minimum_wfs = np.amin(self.inputs_wire_feed_speed_data)
            self.inputs_wire_feed_speed_data = (np.maximum(np.minimum(self.inputs_wire_feed_speed_data, self.maximum_wfs), self.minimum_wfs) - self.minimum_wfs)/(self.maximum_wfs - self.minimum_wfs)
       
            # save the minimum and maximum
            if self.compute_max_min_by_data:
                with open(self.path_name+'maximum_wfs.pkl', 'wb') as file:
                    pickle.dump(self.maximum_wfs, file)

                with open(self.path_name+'minimum_wfs.pkl', 'wb') as file:
                    pickle.dump(self.minimum_wfs, file)

    def load_database(self):
        # load audio data
        self.load_audio_database()

        # load currente data
        self.load_current_database()

        # load Networkvoltage data
        self.load_voltage_database()

        # load travel speed data
        self.load_travel_speed_database()

        # load wire feed speed data
        self.load_wire_feed_speed_database()       

    def create_traning_data(self):
        # create the training database

        # number of elements
        total_elements = len(self.output_data)
        training_number = int(self.training_percentage*total_elements)

        # generating random elements
        random_elements = np.random.choice(total_elements, size=training_number, replace=False)

        # create empty vectors
        self.inputs_audio_pressure_training = np.empty((0,len(self.inputs_audio_pressure[0])))
        self.inputs_audio_fftpower_training = np.empty((0,len(self.inputs_audio_fftpower[0])))
        self.inputs_audio_fftphase_training = np.empty((0,len(self.inputs_audio_fftphase[0])))
        self.inputs_current_data_training = np.empty((0,len(self.inputs_current_data[0])))
        self.inputs_voltage_data_training = np.empty((0,len(self.inputs_voltage_data[0])))
        self.inputs_travel_speed_data_training = np.empty((0,len(self.inputs_travel_speed_data[0])))
        self.inputs_wire_feed_speed_data_training = np.empty((0,len(self.inputs_wire_feed_speed_data[0])))
        self.output_training = np.empty((0,2))

        self.inputs_audio_pressure_test = np.empty((0,len(self.inputs_audio_pressure[0])))
        self.inputs_audio_fftpower_test = np.empty((0,len(self.inputs_audio_fftpower[0])))
        self.inputs_audio_fftphase_test = np.empty((0,len(self.inputs_audio_fftphase[0])))
        self.inputs_current_data_test = np.empty((0,len(self.inputs_current_data[0])))
        self.inputs_voltage_data_test = np.empty((0,len(self.inputs_voltage_data[0])))
        self.inputs_travel_speed_data_test = np.empty((0,len(self.inputs_travel_speed_data[0])))
        self.inputs_wire_feed_speed_data_test = np.empty((0,len(self.inputs_wire_feed_speed_data[0])))
        self.output_test = np.empty((0,2))

        # Convert multi-output labels to a single label
        combined_labels = [f"{label1}-{label2}" for label1, label2 in zip(self.output_data[:, 0], self.output_data[:, 1])]

        # Encode the combined labels
        label_encoder = LabelEncoder()
        encoded_labels = label_encoder.fit_transform(combined_labels)

        # Divisão dos dados usando train_test_split com estratificação
        train_index, test_index = train_test_split(range(len(self.inputs_audio_pressure)), test_size=1-self.training_percentage, random_state=42, stratify=encoded_labels)

        # Split the data into training and testing sets
        self.inputs_audio_pressure_test = self.inputs_audio_pressure[test_index]
        self.inputs_audio_fftpower_test = self.inputs_audio_fftpower[test_index]
        self.inputs_audio_fftphase_test = self.inputs_audio_fftphase[test_index]
        self.inputs_current_data_test = self.inputs_current_data[test_index]
        self.inputs_voltage_data_test = self.inputs_voltage_data[test_index]
        self.inputs_travel_speed_data_test = self.inputs_travel_speed_data[test_index]
        self.inputs_wire_feed_speed_data_test = self.inputs_wire_feed_speed_data[test_index]
        self.output_test = self.output_data[test_index]

        # generating training and test datasets
        for i in range(len(self.output_data)):
            # # conditional
            self.inputs_audio_pressure_training = np.vstack((self.inputs_audio_pressure_training, self.inputs_audio_pressure[i]))
            self.inputs_audio_fftpower_training = np.vstack((self.inputs_audio_fftpower_training, self.inputs_audio_fftpower[i]))
            self.inputs_audio_fftphase_training = np.vstack((self.inputs_audio_fftphase_training, self.inputs_audio_fftphase[i]))
            self.inputs_current_data_training = np.vstack((self.inputs_current_data_training, self.inputs_current_data[i]))
            self.inputs_voltage_data_training = np.vstack((self.inputs_voltage_data_training, self.inputs_voltage_data[i]))
            self.inputs_travel_speed_data_training = np.vstack((self.inputs_travel_speed_data_training, self.inputs_travel_speed_data[i]))
            self.inputs_wire_feed_speed_data_training = np.vstack((self.inputs_wire_feed_speed_data_training, self.inputs_wire_feed_speed_data[i]))
            self.output_training = np.vstack((self.output_training, self.output_data[i]))

if __name__ == '__main__':
    try:
        # ---------------------
        # create a class object
        # ---------------------
        AT = audio_tracking()
        rospy.loginfo(GREEN + "Audio Tracking: " + END + "Begin")

        # ------------------------
        # initializate the thread
        # ------------------------
        nn_thread = threading.Thread(target=AT.neural_network_thread)

        # -------------------
        #  User iteration
        # -------------------
        began = False
        if (AT.type == 1):
            # load database
            rospy.loginfo(GREEN + "Audio Tracking: " + END + "Loading the database.")
            AT.load_database()
            rospy.loginfo(GREEN + "Audio Tracking: " + END + "Database loaded.")

            # generating test data
            rospy.loginfo(GREEN + "Audio Tracking: " + END + "Creating the training dataset")
            AT.create_traning_data()
            rospy.loginfo(GREEN + "Audio Tracking: " + END + "Training dataset was been created.")

            # trainning
            rospy.loginfo(GREEN + "Audio Tracking: " + END + "Start Training!")
            nn_thread.start()

            # begin
            began =  True
        else:
            # prediction model
            rospy.loginfo(GREEN + "Audio Tracking: " + END + "Load prediction model!")
            AT.load_predict_model()

        # -------------------
        #      Main Loop 
        # -------------------
        # rate
        f_hz = float(AT.predict_rate)
        rate = rospy.Rate(f_hz)
        k = 1
        
        # Message 
        rospy.loginfo(GREEN + "Audio Tracking: " + END + "Running")
        
        # loop
        while not rospy.is_shutdown() and (not AT.finish_training) and AT.type != 1:
            # ----------------------
            #  State Machine
            # ----------------------
            if (AT.first_prediciton):
                AT.predition_defect()

            # output screen
            s = 'Time = ' + str(k*(1/f_hz))         # string for output
            sys.stdout.write(s)                     # just print
            sys.stdout.flush()                      # needed for flush when using \x08
            AT.backspace(len(s))

            # iteration 
            k = k + 1

            # sleeping
            rate.sleep()

        # -------------------
        #       Finish
        # ------------------
        if (began):
            nn_thread.join()

        rospy.loginfo(GREEN + "Audio Tracking: " + END + "Finish")

    except rospy.ROSInterruptException:
        pass