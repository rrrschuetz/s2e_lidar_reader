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
    sequences = []  # Temporarily store data for each sequence
    targets = []
    all_lidar_data =[]
    all_color_data = []
    all_targets = []

    with open(filepath, 'r') as file:
        current_sequence = []  # Accumulate data for the current sequence
        current_targets_lstm = []
        current_lidar = []
        current_color = []
        current_targets_cnn = []

        for line in file:
            parts = line.strip().split(',')
            if 'X' in parts[0]:  # Indicates the start of a new sequence
                print('new sequence starting')
                if current_sequence:  # If the current sequence is not empty
                    sequences.append(current_sequence)  # Save the current sequence
                    targets.append(current_targets_lstm)
                    all_lidar_data.extend(current_lidar)
                    all_color_data.extend(current_color)
                    all_targets.extend(current_targets_cnn)
                    current_sequence = []  # Start a new sequence
                    current_targets_lstm = []
                    current_lidar = []
                    current_color = []
                    current_targets_cnn = []
                continue  # Skip the 'Sequence Start' line

            # Extract LIDAR and color data, and concatenate them for each timestep
            target_data = np.array(parts[:2], dtype=np.float32)
            # Apply reciprocal transformation to LIDAR data, avoiding division by zero
            lidar_data = np.array([1/float(x) if float(x) != 0 else 0 for x in parts[2:2432]], dtype=np.float32)
            color_data = np.array(parts[2432:3072], dtype=np.float32)
            combined_data = np.concatenate([lidar_data, color_data])
            current_sequence.append(combined_data)
            current_targets_lstm.append(target_data)
            current_lidar.extend(lidar_data)  # Flatten and collect lidar data
            current_color.extend(color_data)  # Flatten and collect color data
            current_targets_cnn.extend(target_data)

        # Don't forget to append the last sequence if it exists
        if current_sequence:
            sequences.append(current_sequence)
            targets.append(current_targets_lstm)
            all_lidar_data.extend(current_lidar)
            all_color_data.extend(current_color)
            all_targets.extend(current_targets_cnn)

    # Pad the sequences to ensure they have uniform length
    # Determine the maximum sequence length
    max_len = max(len(seq) for seq in sequences)
    # Pad sequences and convert to numpy array for LSTM input
    lstm_inputs = np.array([pad_sequences([seq], maxlen=max_len, dtype='float32', padding='post')[0] for seq in sequences])
    lstm_targets = np.array([pad_sequences([seq], maxlen=max_len, dtype='float32', padding='post')[0] for seq in targets])
    lidar_cnn_inputs = np.array(all_lidar_data).reshape(-1, 2430)  # Reshape for CNN input
    color_cnn_inputs = np.array(all_color_data).reshape(-1, 640)  # Reshape for CNN input
    cnn_targets = np.array(all_targets).reshape(-1,2)

    return lstm_inputs, lstm_targets, lidar_cnn_inputs, color_cnn_inputs, cnn_targets

lstm_inputs, lstm_targets, lidar_cnn_inputs, color_cnn_inputs, cnn_targets = read_and_prepare_data(filepath)
print(f"LSTM Input shape: {lstm_inputs.shape}")
print(f"LSTM Target shape: {lstm_targets.shape}")
print(f"CNN Lidar shape: {lidar_cnn_inputs.shape}")
print(f"CNN Color shape: {color_cnn_inputs.shape}")
print(f"CNN Target shape: {cnn_targets.shape}")

# Splitting the consolidated data
#X_train_seq, X_test_seq, X_train_lidar, X_test_lidar, X_train_color, X_test_color, y_train, y_test = train_test_split(
#    lstm_inputs, lidar_cnn_inputs, color_cnn_inputs, lstm_targets, test_size=0.2, random_state=42)

X_train_seq, X_test_seq, y_train, y_test = train_test_split(
    lstm_inputs, lstm_targets, test_size=0.2, random_state=42)

X_train_lidar, X_test_lidar, X_train_color, X_test_color, y_train_cnn, y_test_cnn = train_test_split(
    lidar_cnn_inputs, color_cnn_inputs, cnn_targets, test_size=0.2, random_state=42)

print("LSTM train data shape:", X_train_seq.shape)
print("LSTM test data shape:", X_test_seq.shape)

print("CNN train data shape:", X_train_lidar.shape)
print("CNN test data shape:", X_test_lidar.shape)

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
    lstm_out = LSTM(32, return_sequences=False)(sequence_input)

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

    # Combining all parts
    combined = concatenate([lstm_out, lidar_flat, color_flat], axis=-1)

    # Dense layers for regression
    dense1 = Dense(64, activation='relu')(combined)
    dense2 = Dense(32, activation='relu')(dense1)
    output = Dense(2, activation='linear')(dense2)  # Assuming 2 regression targets

    model = Model(inputs=[sequence_input, lidar_input, color_input], outputs=output)
    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['mae'])

    return model

# Define the Keras TensorBoard callback
logdir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = TensorBoard(log_dir=logdir)

sequence_shape = Xtrain_seq.shape
lidar_shape = (X_train_lidar.shape[1], 1)
color_shape = (X_train_color.shape[1], 1)
model = create_regression_model(sequence_shape, lidar_shape, color_shape)

# Training the model
history = model.fit(
    [X_train_seq, X_train_lidar, X_train_color],  # Matching the order of inputs in the model
    y_train,  # Targets
    epochs=10,
    batch_size=32,
    validation_data=([X_test_seq, X_test_lidar, X_test_color], y_test),
    callbacks=[EarlyStopping(monitor='val_loss', patience=3)]
)

# 4. Evaluating the model
scores = model.evaluate([X_test_seq, X_test_lidar, X_test_color], y_test, verbose=1)
print(f"Test Loss: {scores[0]}, Test MAE: {scores[1]}")


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
  
