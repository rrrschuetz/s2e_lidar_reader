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

def one_hot_encode_colors(df):
    color_cols = df.filter(regex='^COL').columns
    new_cols = pd.DataFrame()

    for col in color_cols:
        # Create new columns for red and green directly in the new DataFrame
        new_cols[f"{col}_R"] = (df[col] == 2).astype(int)
        new_cols[f"{col}_G"] = (df[col] == 1).astype(int)

    # Concatenate all new columns with the original DataFrame
    df = pd.concat([df, new_cols], axis=1)
    return df

# 1. Preprocess data
data_raw = pd.read_csv('~/test/file.txt')
make_column_names_unique(data_raw)
data_raw = one_hot_encode_colors(data_raw)
data_raw.to_csv('~/test/file_converted.csv', index=False)
print("Raw data columns:", data_raw.columns)
print("Raw data shape:", data_raw.shape)

# Split data into train and test sets
train, test = train_test_split(data_raw, test_size=0.2)
print("Train data shape:", train.shape)
print("Test data shape:", test.shape)

# Split the training and testing data into input and target
cols_to_include = [col for col in train.columns if col not in ['X', 'Y']]
x_train = train[cols_to_include].values
y_train = train[['X', 'Y']].values
x_test = test[cols_to_include].values
y_test = test[['X', 'Y']].values
print("x_train shape:", x_train.shape)
print("y_train shape:", y_train.shape)
print("x_test shape:", x_test.shape)
print("y_test shape:", y_test.shape)

# Standardization
scaler = StandardScaler().fit(x_train)
print("Scaler fitted on x_train")
x_train = scaler.transform(x_train)
x_test = scaler.transform(x_test)
print("After standardization, x_train shape:", x_train.shape)
print("After standardization, x_test shape:", x_test.shape)

# Reshape the data to 3D - (batch_size, steps, 1)
x_train = x_train.reshape(x_train.shape[0], x_train.shape[1], 1)
x_test = x_test.reshape(x_test.shape[0], x_test.shape[1], 1)
print("After reshaping, x_train shape:", x_train.shape)
print("After reshaping, x_test shape:", x_test.shape)

# 2. Define the 1D CNN model

#def create_cnn_model(input_shape):
#    model = tf.keras.models.Sequential()
#    model.add(Conv1D(64, kernel_size=5, activation='relu', input_shape=input_shape))
#    model.add(MaxPooling1D(pool_size=2))
#    model.add(Conv1D(128, kernel_size=5, activation='relu'))
#    model.add(MaxPooling1D(pool_size=2))
#    model.add(Flatten())
#    model.add(Dense(64, activation='relu'))
#    model.add(Dense(32, activation='relu'))
#    model.add(Dense(2))
#
#    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
#    return modeldef create_cnn_model(input_shape):

def create_cnn_model(input_shape):
    print("Creating model with input shape:", input_shape)
    model = tf.keras.models.Sequential()
    model.add(Conv1D(64, kernel_size=5, activation='relu', input_shape=input_shape))
#    model.add(BatchNormalization())  # Add Batch Normalization
    model.add(MaxPooling1D(pool_size=2))

    model.add(Conv1D(128, kernel_size=5, activation='relu', kernel_regularizer=l2(0.01)))  # Add L2 Regularization
#    model.add(BatchNormalization())  # Add Batch Normalization
    model.add(MaxPooling1D(pool_size=2))
#    model.add(Dropout(0.5))  # Add Dropout

#    model.add(LSTM(50, return_sequences=True))  # Add an LSTM layer
#    model.add(LSTM(50))  # Another LSTM layer, you can add as many as you need

    model.add(Flatten())
    model.add(Dense(64, activation='relu'))
#    model.add(Dropout(0.5))  # Add Dropout

    model.add(Dense(32, activation='relu'))
    model.add(Dense(2))

    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
    return model

# Create EarlyStopping callback
early_stopping_callback = EarlyStopping(monitor='val_loss', patience=2)

# Define the Keras TensorBoard callback
logdir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = TensorBoard(log_dir=logdir)

input_shape = (x_train.shape[1], 1)
model = create_cnn_model(input_shape)

# 3. Train the model
model.summary()
#history = model.fit(x_train, y_train, epochs=15, validation_split=0.2, batch_size=32)
# with early stopping
history = model.fit(x_train, y_train, epochs=45, validation_split=0.2, batch_size=32, callbacks=[early_stopping_callback,tensorboard_callback])

# 4. Evaluate the model
loss, acc = model.evaluate(x_test, y_test, verbose=2)
print(f"Model's accuracy: {100 * acc:.2f}%")

# 5. Save the model and the scaler for standardization
model.save('/home/rrrschuetz/test/model')
with open('/home/rrrschuetz/test/scaler.pkl', 'wb') as f:
    pickle.dump(scaler, f)

# Convert the model.
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# Save the model to disk
with open('/home/rrrschuetz/test/model.tflite', 'wb') as f:
    f.write(tflite_model)
