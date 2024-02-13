import datetime
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input
from tensorflow.keras.layers import Concatenate, Layer
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Flatten, Dense, LSTM
from tensorflow.keras.layers import Dropout, BatchNormalization
from tensorflow.keras.layers import LSTM  # Import LSTM layer
from tensorflow.keras.regularizers import l2
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.callbacks import TensorBoard
from tensorflow.keras.preprocessing.sequence import pad_sequences

import pickle  # For saving the scaler

filepath = '/home/rrrschuetz/test/file_p.txt'

def apply_reciprocal_to_scan(df):
    scan_cols = df.filter(regex='^SCAN').columns
    for col in scan_cols:
        # Apply the reciprocal transformation, avoiding division by zero
        df[col] = df[col].apply(lambda x: 1/x if x != 0 else 0)
    return df

import numpy as np
from tensorflow.keras.preprocessing.sequence import pad_sequences

def read_and_prepare_data(filepath):
    with open(filepath, 'r') as file:
        sequences = []
        all_lidar_data = []
        all_color_data = []
        targets = []

        current_sequence_lidar = []
        current_sequence_color = []

        for line in file:
            parts = line.strip().split(',')

            # Check if it's a header or sequence start line
            if 'X' in parts[0] and 'Y' in parts[1]:  # Assuming 'X,Y' are present in sequence start
                if current_sequence_lidar and current_sequence_color:  # Save previous sequence if exists
                    sequences.append({
                        'lidar': np.array(current_sequence_lidar, dtype=np.float32),
                        'color': np.array(current_sequence_color, dtype=np.float32)
                    })
                current_sequence_lidar = []
                current_sequence_color = []

            targets.append([float(parts[0]), float(parts[1])])
            # Process lidar and color data for the current sequence
            lidar_data = np.array(parts[2:2432], dtype=np.float32)  # Adjust indices as needed
            color_data = np.array(parts[2432:], dtype=np.float32)  # Adjust indices as needed
            current_sequence_lidar.append(lidar_data)
            current_sequence_color.append(color_data)

            # Add to consolidated data
            all_lidar_data.extend(lidar_data)
            all_color_data.extend(color_data)

        # Don't forget the last sequence
        if current_sequence_lidar and current_sequence_color:
            sequences.append({
                'lidar': np.array(current_sequence_lidar, dtype=np.float32),
                'color': np.array(current_sequence_color, dtype=np.float32)
            })

    # Determine max sequence length for padding
    max_len = max(max(len(seq['lidar']), len(seq['color'])) for seq in sequences)

    # Pad sequences
    for seq in sequences:
        seq['lidar'] = pad_sequences(seq['lidar'], maxlen=max_len, padding='post')
        seq['color'] = pad_sequences(seq['color'], maxlen=max_len, padding='post')

    # Prepare LSTM inputs as 3D array and targets
    lstm_inputs = np.array([np.hstack((seq['lidar'], seq['color'])) for seq in sequences])
    targets = np.array(targets, dtype=np.float32)

    # Prepare consolidated data for CNN inputs
    all_lidar_data = np.array(all_lidar_data, dtype=np.float32).reshape(-1, 1)
    all_color_data = np.array(all_color_data, dtype=np.float32).reshape(-1, 1)

    return lstm_inputs, targets, all_lidar_data, all_color_data

lstm_inputs, targets, all_lidar_data, all_color_data = read_and_prepare_data(filepath)

print("LSTM Input shape:", sequences.shape)
print("Consolidated Lidar shape:", all_lidar.shape)
print("Consolidated Color shape:", all_color.shape)
print("Targets shape:", targets.shape)

# Example split (ensure it aligns with your model's requirements)
X_seq = np.array([seq['lidar'] for seq in sequences])  # Assuming you want to use lidar data for LSTM
X_lidar = consolidated_lidar
X_color = consolidated_color
Y = targets

# Splitting the consolidated data
X_train_lidar, X_test_lidar, X_train_color, X_test_color, y_train, y_test = train_test_split(
    X_lidar, X_color, Y, test_size=0.2, random_state=42)

# Assuming a simple split for sequences for demonstration; adjust as needed for your temporal data
X_train_seq, X_test_seq = train_test_split(X_seq, test_size=0.2, random_state=42)

#data_raw = apply_reciprocal_to_scan(data_raw)
#print("Raw data columns:", data_raw.columns)
#print("Raw data shape:", data_raw.shape)

print("Train data shape:", X_train_lidar.shape)
print("Test data shape:", X_test_lidar.shape)

# Standardization
#scaler_lidar = StandardScaler().fit(train_lidar.values)
#print("Scaler fitted on x_train")
#train_lidar = scaler_lidar.transform(train_lidar.values).astype(np.float32)
#test_lidar = scaler_lidar.transform(test_lidar.values).astype(np.float32)

# 2. Define the 1D CNN model
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
    # Sequence Input for LSTM
    sequence_input = Input(shape=sequence_shape, name="sequence_input")
    lstm_out = LSTM(32,return_sequences=False)(sequence_input)

    # LIDAR Input for CNN
    lidar_input = Input(shape=lidar_shape, name="lidar_input")
    lidar_conv1 = Conv1D(filters=64, kernel_size=3, activation='relu')(lidar_input)
    lidar_pool1 = MaxPooling1D(pool_size=2)(lidar_conv1)
    lidar_flat = Flatten()(lidar_pool1)

    # Color Input for CNN
    color_input = Input(shape=color_shape, name="color_input")
    color_conv1 = Conv1D(filters=32, kernel_size=3, activation='relu')(color_input)
    color_pool1 = MaxPooling1D(pool_size=2)(color_conv1)
    color_flat = Flatten()(color_pool1)

    # Combine all outputs
    combined = Concatenate()([lstm_out, lidar_flat, color_flat])

    # Fully connected layers
    dense1 = Dense(64, activation='relu')(combined)
    output = Dense(2, activation='linear')(dense1)  # Output layer for regression

    model = Model(inputs=[sequence_input, lidar_input, color_input], outputs=output)

    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['mae'])  # Compile for regression

    return model

# Define the Keras TensorBoard callback
logdir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = TensorBoard(log_dir=logdir)

max_sequence_length = max([seq['lidar'].shape[0] for seq in sequences])
print("max_sequence_length ",max_sequence_length)
sequence_shape = (None, max_sequence_length,X_train_lidar.shape[1])
lidar_shape = (X_train_lidar.shape[1], 1)
color_shape = (X_train_color.shape[1], 1)
model = create_regression_model(sequence_shape, lidar_shape, color_shape)

# 3. Train the model
model.summary()

history = model.fit(
    [X_train_seq, X_train_lidar, X_train_color],  # input data
    y_train,  # target data (heading and power)
    epochs=10,  # adjust based on convergence
    batch_size=32,  # adjust based on your dataset size and memory capacity
    validation_data=([X_test_seq, X_test_lidar, X_test_color], y_test),  # data for evaluation
    callbacks=[EarlyStopping(monitor='val_loss', patience=3)]  # early stopping
)

# 4. Evaluate the model
scores = model.evaluate([X_test_seq, X_test_lidar, X_test_color], y_test, verbose=1)
print(f"Test MSE: {scores[0]}, Test MAE: {scores[1]}")

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
  
