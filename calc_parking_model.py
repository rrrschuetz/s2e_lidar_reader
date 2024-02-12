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
import pickle  # For saving the scaler

filepath = '/home/rrrschuetz/test/file_p.txt'

def apply_reciprocal_to_scan(df):
    scan_cols = df.filter(regex='^SCAN').columns
    for col in scan_cols:
        # Apply the reciprocal transformation, avoiding division by zero
        df[col] = df[col].apply(lambda x: 1/x if x != 0 else 0)
    return df

def parse_header(header_line):
    headers = header_line.strip().split(',')
    lidar_indices = [i for i, h in enumerate(headers) if h.startswith('SCAN')]
    color_indices = [i for i, h in enumerate(headers) if h.startswith('COL')]
    return lidar_indices, color_indices

def read_sequences_and_separate_data(filepath):
    with open(filepath, 'r') as file:
        header_line = next(file).strip()  # Read the first line to get the headers
        lidar_indices, color_indices = parse_header(header_line)

        sequences = []  # To store sequence-wise data
        consolidated_lidar = []  # To store all LIDAR data across sequences
        consolidated_color = []  # To store all color data across sequences

        current_sequence = {'lidar': [], 'color': []}  # Temp storage for the current sequence

        for line in file:
            if line.startswith('X,Y'):  # Start of a new sequence
                if current_sequence['lidar']:  # If there's data in the current sequence
                    sequences.append(current_sequence)  # Save the sequence
                    current_sequence = {'lidar': [], 'color': []}  # Start a new sequence
                continue  # Skip the header line

            parts = line.strip().split(',')
            lidar_row = [float(parts[i]) if parts[i] != 'nan' else 0.0 for i in lidar_indices]
            color_row = [float(parts[i]) if parts[i] != 'nan' else 0.0 for i in color_indices]

            # Add data to the current sequence
            current_sequence['lidar'].append(lidar_row)
            current_sequence['color'].append(color_row)

            # Also add data to the consolidated arrays
            consolidated_lidar.append(lidar_row)
            consolidated_color.append(color_row)

        # Add the last sequence if not empty
        if current_sequence['lidar']:
            sequences.append(current_sequence)

    # Convert everything to numpy arrays for easier handling in TensorFlow/Keras
    sequences = [{'lidar': np.array(seq['lidar']), 'color': np.array(seq['color'])} for seq in sequences]
    consolidated_lidar = np.array(consolidated_lidar)
    consolidated_color = np.array(consolidated_color)

    return sequences, consolidated_lidar, consolidated_color

sequences, consolidated_lidar, consolidated_color = read_sequences_and_separate_data(filepath)

X_train_seq, X_test_seq, X_train_lidar, X_test_lidar, X_train_color, X_test_color, y_train, y_test = train_test_split(
    padded_sequences, lidar_data, color_data, labels, test_size=0.2, random_state=42
)

data_raw = apply_reciprocal_to_scan(data_raw)
print("Raw data columns:", data_raw.columns)
print("Raw data shape:", data_raw.shape)

# Split data into train and test sets
train, test = train_test_split(data_raw, test_size=0.2)
print("Train data shape:", train.shape)
print("Test data shape:", test.shape)

train_lidar = train.iloc[:, 2:2432]
test_lidar = test.iloc[:, 2:2432]
train_color = train.iloc[:, -640:]
test_color = test.iloc[:, -640:]
y_train = train.iloc[:, :2]
y_test = test.iloc[:, :2]

# Standardization
scaler_lidar = StandardScaler().fit(train_lidar.values)
print("Scaler fitted on x_train")
train_lidar = scaler_lidar.transform(train_lidar.values).astype(np.float32)
test_lidar = scaler_lidar.transform(test_lidar.values).astype(np.float32)

# Convert to numpy arrays and reshape as needed for LIDAR data
#x_train_lidar = train_lidar.values.reshape(train_lidar.shape[0], train_lidar.shape[1], 1)
x_train_lidar = train_lidar.reshape(train_lidar.shape[0], train_lidar.shape[1], 1)
#x_test_lidar = test_lidar.values.reshape(test_lidar.shape[0], test_lidar.shape[1], 1
x_test_lidar = test_lidar.reshape(test_lidar.shape[0], test_lidar.shape[1], 1)
print("After standardization, x_train lidar shape:", x_train_lidar.shape)
print("After standardization, x_test lidar shape:", x_test_lidar.shape)

train_color = train_color.values.astype(np.float32)
test_color = test_color.values.astype(np.float32)
x_train_color = train_color.reshape(train_color.shape[0], train_color.shape[1], 1)
x_test_color = test_color.reshape(test_color.shape[0], test_color.shape[1], 1)
print("After standardization, x_train color shape:", x_train_color.shape)
print("After standardization, x_test color shape:", x_test_color.shape)

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
    lstm_out = LSTM(32)(sequence_input)

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
    combined = concatenate([lstm_out, lidar_flat, color_flat])

    # Fully connected layers
    dense1 = Dense(64, activation='relu')(combined)
    output = Dense(2, activation='linear')(dense1)  # Output layer for regression

    model = Model(inputs=[sequence_input, lidar_input, color_input], outputs=output)

    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['mae'])  # Compile for regression

    return model

# Assuming shapes from your data prep
sequence_shape = (None, 20)  # Update according to your data
lidar_shape = (None, 1)  # Update according to your data
color_shape = (None, 1)  # Update according to your data
model = create_regression_model(sequence_shape, lidar_shape, color_shape)

# Create EarlyStopping callback
early_stopping_callback = EarlyStopping(monitor='val_loss', patience=3)

# Define the Keras TensorBoard callback
logdir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = TensorBoard(log_dir=logdir)

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
  
