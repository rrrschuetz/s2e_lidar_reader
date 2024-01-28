import datetime
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Flatten, Dense
from tensorflow.keras.layers import Dropout, BatchNormalization
from tensorflow.keras.layers import LSTM  # Import LSTM layer
from tensorflow.keras.regularizers import l2
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.callbacks import TensorBoard
import pickle  # For saving the scaler

#import matplotlib.pyplot as plt

# How to call tensorboard:
# tensorboard --logdir logs/fit
# http://localhost:6006/ to see various metrics and details

def make_column_names_unique(df):
    cols = pd.Series(df.columns)
    for dup in cols[cols.duplicated()].unique():
        cols[cols[cols == dup].index.values.tolist()] = [dup + '_' + str(i) if i != 0 else dup for i in range(sum(cols == dup))]
    df.columns = cols

def apply_reciprocal_to_scan(df):
    scan_cols = df.filter(regex='^SCAN').columns
    for col in scan_cols:
        # Apply the reciprocal transformation, avoiding division by zero
        df[col] = df[col].apply(lambda x: 1/x if x != 0 else 0)
    return df

# 1. Preprocess data
data_raw = pd.read_csv('~/test/file.txt')
make_column_names_unique(data_raw)
data_raw = apply_reciprocal_to_scan(data_raw)
print("Raw data columns:", data_raw.columns)
print("Raw data shape:", data_raw.shape)

# Split data into train and test sets
#train, test = train_test_split(data_raw, test_size=0.2)
#print("Train data shape:", train.shape)
#print("Test data shape:", test.shape)

lidar_data = data_raw.iloc[:, 2:1622]
color_data = data_raw.iloc[:, 1622:1623]

# Split into train and test sets for both LIDAR and color data
train_lidar, test_lidar = train_test_split(lidar_data, test_size=0.2)
train_color, test_color = train_test_split(color_data, test_size=0.2)
y_train = train[['X', 'Y']].values

# Split the training and testing data into input and target
#cols_to_include = [col for col in train.columns if col not in ['X', 'Y']]
#x_train = train[cols_to_include].values
#y_train = train[['X', 'Y']].values
#x_test = test[cols_to_include].values
#y_test = test[['X', 'Y']].values
#print("x_train shape:", x_train.shape)
#print("y_train shape:", y_train.shape)
#print("x_test shape:", x_test.shape)
#print("y_test shape:", y_test.shape)

# Standardization
scaler_lidar = StandardScaler().fit(x_train_lidar)
scaler_color = StandardScaler().fit(x_train_color)
print("Scaler fitted on x_train")
x_train_lidar = scaler_lidar.transform(x_train_lidar)
x_train_color = scaler_color.transform(x_train_color)
x_test_lidar = scaler_lidar.transform(x_test_lidar)
x_test_color = scaler_color.transform(x_test_color)
#print("After standardization, x_train shape:", x_train.shape)
#print("After standardization, x_test shape:", x_test.shape)

# Reshape the data to 3D - (batch_size, steps, 1)
#x_train = x_train.reshape(x_train.shape[0], x_train.shape[1], 1)
#x_test = x_test.reshape(x_test.shape[0], x_test.shape[1], 1)
#print("After reshaping, x_train shape:", x_train.shape)
#print("After reshaping, x_test shape:", x_test.shape)

# Convert to numpy arrays and reshape as needed for LIDAR data
x_train_lidar = train_lidar.values.reshape(train_lidar.shape[0], train_lidar.shape[1], 1)
x_test_lidar = test_lidar.values.reshape(test_lidar.shape[0], test_lidar.shape[1], 1)
# Reshape color data as it's a single channel
x_train_color = train_color.values.reshape(train_color.shape[0], 1)
x_test_color = test_color.values.reshape(test_color.shape[0], 1)

# 2. Define the 1D CNN model

#def create_cnn_model(input_shape):
#    print("Creating model with input shape:", input_shape)
#    model = tf.keras.models.Sequential()
#    model.add(Conv1D(64, kernel_size=5, activation='relu', input_shape=input_shape))
##    model.add(BatchNormalization())  # Add Batch Normalization
#    model.add(MaxPooling1D(pool_size=2))
#
#    model.add(Conv1D(128, kernel_size=5, activation='relu', kernel_regularizer=l2(0.01)))  # Add L2 Regularization
##    model.add(BatchNormalization())  # Add Batch Normalization
#    model.add(MaxPooling1D(pool_size=2))
##    model.add(Dropout(0.5))  # Add Dropout
#
##    model.add(LSTM(50, return_sequences=True))  # Add an LSTM layer
##    model.add(LSTM(50))  # Another LSTM layer, you can add as many as you need
#
#    model.add(Flatten())
#    model.add(Dense(64, activation='relu'))
##    model.add(Dropout(0.5))  # Add Dropout
#
#    model.add(Dense(32, activation='relu'))
#    model.add(Dense(2))
#
#    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
#    return model
    
def create_cnn_model(lidar_input_shape, color_input_shape):
    # LIDAR data path
    lidar_input = Input(shape=lidar_input_shape)
    lidar_path = Conv1D(64, kernel_size=5, activation='relu')(lidar_input)
    lidar_path = MaxPooling1D(pool_size=2)(lidar_path)
    lidar_path = Conv1D(128, kernel_size=5, activation='relu', kernel_regularizer=l2(0.01))(lidar_path)
    lidar_path = MaxPooling1D(pool_size=2)(lidar_path)
    lidar_path = Flatten()(lidar_path)

    # Color data path
    color_input = Input(shape=color_input_shape)
    color_path = Dense(32, activation='relu')(color_input)  # Simple dense layer for color data
    color_path = Flatten()(color_path)  # Flatten if necessary

    # Concatenation
    concatenated = Concatenate()([lidar_path, color_path])

    # Further processing
    combined = Dense(64, activation='relu')(concatenated)
    combined = Dense(32, activation='relu')(combined)
    output = Dense(2)(combined)

    model = Model(inputs=[lidar_input, color_input], outputs=output)
    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
    return model

# Create EarlyStopping callback
early_stopping_callback = EarlyStopping(monitor='val_loss', patience=2)

# Define the Keras TensorBoard callback
logdir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = TensorBoard(log_dir=logdir)

# Input shape for LIDAR data  (number of channels, 1)
lidar_input_shape = (x_train_lidar.shape[1], 1)

# Input shape for color data is simply (1,)
color_input_shape = (1,)

# Create the model
model = create_cnn_model(lidar_input_shape, color_input_shape)

#model = create_cnn_model(input_shape)

# 3. Train the model
model.summary()
#history = model.fit(x_train, y_train, epochs=15, validation_split=0.2, batch_size=32)
# with early stopping
#history = model.fit(x_train, y_train, epochs=45, validation_split=0.2, batch_size=32, callbacks=[early_stopping_callback,tensorboard_callback])
# Assuming x_train_lidar, x_train_color are your LIDAR and color training data,
# and y_train is your training labels

history = model.fit(
    [x_train_lidar, x_train_color],  # Input data
    y_train,                         # Labels
    epochs=45,                       # Number of epochs to train for
    batch_size=32,                   # Batch size
    validation_split=0.2,  # Percentage of data to use for validation
    callbacks=[early_stopping_callback,tensorboard_callback]       # Optional callbacks, like EarlyStopping
)

# 4. Evaluate the model
#loss, acc = model.evaluate(x_test, y_test, verbose=2)
#print(f"Model's accuracy: {100 * acc:.2f}%")

loss, accuracy = model.evaluate(
    [x_test_lidar, x_test_color], 
    y_test
)
print(f"Test Loss: {loss}")
print(f"Test Accuracy: {accuracy}")

# 5. Save the model and the scaler for standardization
model.save('/home/rrrschuetz/test/model')
with open('/home/rrrschuetz/test/scaler_lidar.pkl', 'wb') as f:
    pickle.dump(scaler_lidar, f)
with open('/home/rrrschuetz/test/scaler_color.pkl', 'wb') as f:
    pickle.dump(scaler_color, f)
    
# Convert the model.
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# Save the model to disk
with open('/home/rrrschuetz/test/model.tflite', 'wb') as f:
    f.write(tflite_model)
