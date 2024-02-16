import datetime
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Concatenate, concatenate, Input, Layer
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Flatten, Dense, LSTM
from tensorflow.keras.layers import Dropout, BatchNormalization
from tensorflow.keras.regularizers import l2
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.callbacks import TensorBoard
from tensorflow.keras.preprocessing.sequence import pad_sequences
import pickle  # For saving the scaler

filepath = '/home/rrrschuetz/test/file_p.txt'
def read_and_prepare_data(filepath):
    sequences = []
    lstm_targets = []
    all_lidar_data = []
    all_color_data = []
    cnn_targets = []

    with open(filepath, 'r') as file:
        current_sequence = []
        current_targets = []
        current_lidar = []
        current_color = []

        for line in file:
            parts = line.strip().split(',')
            if 'X' in parts[0]:  # New sequence indicator
                if current_sequence:  # If there's an existing sequence
                    sequences.append(current_sequence)
                    lstm_targets.append(current_targets)  # LSTM targets
                    all_lidar_data.append(current_lidar)
                    all_color_data.append(current_color)
                    cnn_targets.append(current_targets)  # CNN targets, assuming one target set per sequence
                    # Reset for the next sequence
                    current_sequence = []
                    current_targets = []
                    current_lidar = []
                    current_color = []
                continue

            # Processing sequence data
            target_data = np.array(parts[:2], dtype=np.float32)
            lidar_data = np.array([1/float(x) if float(x) != 0 else 0 for x in parts[2:2432]], dtype=np.float32)
            color_data = np.array(parts[2432:3072], dtype=np.float32)
            current_sequence.append(np.concatenate([lidar_data, color_data]))
            current_targets.append(target_data)
            current_lidar.append(lidar_data)
            current_color.append(color_data)

        # Save the last sequence
        if current_sequence:
            sequences.append(current_sequence)
            lstm_targets.append(current_targets)
            all_lidar_data.append(current_lidar)
            all_color_data.append(current_color)
            cnn_targets.append(current_targets)

    # Preparing LSTM inputs and targets
    max_len = max(len(seq) for seq in sequences)
    lstm_inputs = np.array([pad_sequences([seq], maxlen=max_len, dtype='float32', padding='post')[0] for seq in sequences])
    lstm_targets_padded = np.array([pad_sequences([targ], maxlen=max_len, dtype='float32', padding='post')[0] for targ in lstm_targets])

    # Preparing CNN inputs; flattening sequences
    lidar_cnn_inputs = np.concatenate([np.array(seq).flatten() for seq in all_lidar_data]).reshape(-1, 2430)
    color_cnn_inputs = np.concatenate([np.array(seq).flatten() for seq in all_color_data]).reshape(-1, 640)

    # Preparing CNN targets; ensuring uniform structure
    cnn_targets_flat = np.concatenate(cnn_targets).reshape(-1, 2)

    return lstm_inputs, lstm_targets_padded, lidar_cnn_inputs, color_cnn_inputs, cnn_targets_flat

# Assuming 'filepath' is defined and points to your data file
lstm_inputs, lstm_targets, lidar_cnn_inputs, color_cnn_inputs, cnn_targets = read_and_prepare_data(filepath)

# Splitting the consolidated data
#X_train_seq, X_test_seq, X_train_lidar, X_test_lidar, X_train_color, X_test_color, y_train, y_test = train_test_split(
#    lstm_inputs, lidar_cnn_inputs, color_cnn_inputs, lstm_targets, test_size=0.2, random_state=42)

X_train_seq, X_test_seq, y_train, y_test = train_test_split(
    lstm_inputs, lstm_targets, test_size=0.2, random_state=42)

X_train_lidar, X_test_lidar, X_train_color, X_test_color, y_train_cnn, y_test_cnn = train_test_split(
    lidar_cnn_inputs, color_cnn_inputs, cnn_targets, test_size=0.2, random_state=42)

# Standardization
#scaler_lidar = StandardScaler().fit(train_lidar.values)
#print("Scaler fitted on x_train")
#train_lidar = scaler_lidar.transform(train_lidar.values).astype(np.float32)
#test_lidar = scaler_lidar.transform(test_lidar.values).astype(np.float32)

# Define the 1D CNN model
class WeightedConcatenate(Layer):
    def __init__(self, weight_lidar=0.5, weight_color=0.5, **kwargs):
         super(WeightedConcatenate, self).__init__(**kwargs)
         self.weight_lidar = weight_lidar
         self.weight_color = weight_color
    def call(self, inputs):
         lidar, color = inputs
         return tf.concat([self.weight_lidar * lidar, self.weight_color * color], axis=-1)

def create_cnn_model(lidar_input_shape, color_input_shape):

    # LIDAR data path
    lidar_input = Input(shape=lidar_input_shape)
    lidar_path = Conv1D(64, kernel_size=5, activation='relu')(lidar_input)
    lidar_path = MaxPooling1D(pool_size=2)(lidar_path)
    lidar_path = Conv1D(128, kernel_size=5, activation='relu', kernel_regularizer=l2(0.01))(lidar_path)
    lidar_path = MaxPooling1D(pool_size=2)(lidar_path)
    lidar_path = Flatten()(lidar_path)

    # COLOR data path
    color_input = Input(shape=color_input_shape)
    color_path = Dense(64, activation='relu')(color_input)
    color_path = Dropout(0.3)(color_path)  # Use dropout
    color_path = Dense(128, activation='relu', kernel_regularizer=l2(0.01))(color_path)  # Regularization
    color_path = Flatten()(color_path)

    # Concatenation
    concatenated = WeightedConcatenate(weight_lidar=0.2, weight_color=0.8)([lidar_path, color_path])

    # Further processing
    combined = Dense(64, activation='relu')(concatenated)
    combined = Dense(64, activation='relu')(combined)
    combined = Dense(32, activation='relu')(combined)
    output = Dense(2)(combined)

    model = Model(inputs=[lidar_input, color_input], outputs=output)
    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
    return model

def create_regression_model(sequence_shape, lidar_shape, color_shape):
    # LSTM Part
    sequence_input = Input(shape=sequence_shape, name="sequence_input")
    lstm_layer = LSTM(32, return_sequences=False)(sequence_input)
    lstm_output = Dense(2, activation='linear', name='lstm_output')(lstm_layer)  # Adjust the units based on your target

    # CNN Part for LIDAR data
    lidar_input = Input(shape=lidar_shape, name="lidar_input")
    lidar_conv = Conv1D(filters=64, kernel_size=3, activation='relu')(lidar_input)
    lidar_flat = Flatten()(lidar_conv)

    # CNN Part for Color data
    color_input = Input(shape=color_shape, name="color_input")
    color_conv = Conv1D(filters=32, kernel_size=3, activation='relu')(color_input)
    color_flat = Flatten()(color_conv)

    # Combining CNN parts
    cnn_combined = concatenate([lidar_flat, color_flat], axis=-1)
    cnn_output = Dense(2, activation='linear', name='cnn_output')(cnn_combined)  # Adjust the units based on your target

    model = Model(inputs=[sequence_input, lidar_input, color_input], outputs=[lstm_output, cnn_output])
    model.compile(optimizer='adam',
                  loss={'lstm_output': 'mean_squared_error', 'cnn_output': 'mean_squared_error'},
                  metrics={'lstm_output': ['mae'], 'cnn_output': ['mae']})

    return model

# Define the Keras TensorBoard callback
logdir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = TensorBoard(log_dir=logdir)

print("X_train_seq shape:", X_train_seq.shape)  # LSTM input
print("X_train_lidar shape:", X_train_lidar.shape)  # CNN input for LIDAR
print("X_train_color shape:", X_train_color.shape)  # CNN input for Color
print("y_train shape:", y_train.shape)  # LSTM targets
print("y_train_cnn shape:", y_train_cnn.shape)  # CNN targets

sequence_shape = (X_train_seq.shape[1],X_train_seq.shape[2])
lidar_shape = (X_train_lidar.shape[1], 1)
color_shape = (X_train_color.shape[1], 1)
model = create_regression_model(sequence_shape, lidar_shape, color_shape)

history = model.fit(
    [X_train_seq, X_train_lidar, X_train_color],  # Input data
    {'lstm_output': y_train, 'cnn_output': y_train_cnn},  # Target data
    epochs=10,
    batch_size=32,
    validation_data=([X_test_seq, X_test_lidar, X_test_color],
    {'lstm_output': y_test, 'cnn_output': y_test_cnn}),
    callbacks=[EarlyStopping(monitor='val_loss', patience=3)]
)

# Evaluating the model
scores = model.evaluate([X_test_seq, X_test_lidar, X_test_color], {'lstm_output': y_test, 'cnn_output': y_test_cnn}, verbose=1)
print(f"Test Scores: {scores}")

# 5. Save the model and the scaler for standardization
model.save('/home/rrrschuetz/test/model_p')
with open('/home/rrrschuetz/test/scaler_p.pkl', 'wb') as f:
    pickle.dump(scaler_lidar, f)

# Convert the model.
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# Save the model to disk
with open('/home/rrrschuetz/test/model_p.tflite', 'wb') as f:
    f.write(tflite_model)
  
