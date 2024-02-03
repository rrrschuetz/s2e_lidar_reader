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
data_raw = pd.read_csv('~/test/file_p.txt')
make_column_names_unique(data_raw)
data_raw = apply_reciprocal_to_scan(data_raw)
print(data_raw.columns)

# Split data into train and test sets
train, test = train_test_split(data_raw, test_size=0.2)

# Split the training and testing data into input and target
cols_to_include = [col for col in train.columns if col not in ['X', 'Y']]
x_train = train[cols_to_include].values
y_train = train[['X', 'Y']].values
x_test = test[cols_to_include].values
y_test = test[['X', 'Y']].values

# Standardization
scaler = StandardScaler().fit(x_train)
x_train = scaler.transform(x_train)
x_test = scaler.transform(x_test)

# Reshape the data to 3D - (batch_size, steps, 1)
x_train = x_train.reshape(x_train.shape[0], x_train.shape[1], 1)
x_test = x_test.reshape(x_test.shape[0], x_test.shape[1], 1)
print("After standardization, x_train shape:", x_train.shape)
print("After standardization, x_test shape:", x_test.shape)

# 2. Define the 1D CNN model
def create_cnn_model(input_shape):
    model = tf.keras.models.Sequential()
    model.add(Conv1D(64, kernel_size=5, activation='relu', input_shape=input_shape))
    model.add(MaxPooling1D(pool_size=2))
    model.add(Conv1D(128, kernel_size=5, activation='relu', kernel_regularizer=l2(0.01)))  # Add L2 Regularization
    model.add(MaxPooling1D(pool_size=2))
    model.add(Flatten())
    model.add(Dense(64, activation='relu'))
    model.add(Dense(32, activation='relu'))
    model.add(Dense(2))
    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
    return model

# Create EarlyStopping callback
early_stopping_callback = EarlyStopping(monitor='val_loss', patience=3)

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
model.save('/home/rrrschuetz/test/model_p')
with open('/home/rrrschuetz/test/scaler_p.pkl', 'wb') as f:
    pickle.dump(scaler, f)

# Convert the model.
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# Save the model to disk
with open('/home/rrrschuetz/test/model_p.tflite', 'wb') as f:
    f.write(tflite_model)
  
