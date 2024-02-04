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

# Replace Inf values with NaN so they can be handled uniformly
#data_raw[data_raw > 3] = np.nan
#data_raw[data_raw == np.inf] = np.nan
#data_raw[data_raw == np.nan] = 0.0

#data_raw.replace([data_raw > 3], np.nan, inplace=True)
#data_raw.replace([np.inf, -np.inf], np.nan, inplace=True)

# Check for columns that are entirely NaN and decide on an action
#all_nan_columns = data_raw.columns[data_raw.isnull().all()]
# For simplicity, let's fill these with 0 (or choose another strategy as needed)
#data_raw[all_nan_columns] = data_raw[all_nan_columns].fillna(0)

# Now, impute NaN values for other columns with their mean
#for column in data_raw.columns:
#    if column not in all_nan_columns:  # Skip already handled all-NaN columns
#        data_raw[column].fillna(data_raw[column].mean(), inplace=True)

print("NaN ",data_raw.isnull().values.any())
print("inf ",np.isinf(data_raw).values.any())

data_raw = apply_reciprocal_to_scan(data_raw)
print("Raw data columns:", data_raw.columns)
print("Raw data shape:", data_raw.shape)

# Split data into train and test sets
train, test = train_test_split(data_raw, test_size=0.2)
print("Train data shape:", train.shape)
print("Test data shape:", test.shape)

train_lidar = train.iloc[:, 2:1622]
test_lidar = test.iloc[:, 2:1622]
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

# Create EarlyStopping callback
early_stopping_callback = EarlyStopping(monitor='val_loss', patience=3)

# Define the Keras TensorBoard callback
logdir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = TensorBoard(log_dir=logdir)

# Input shape for LIDAR data  (number of channels, 1)
lidar_input_shape = (x_train_lidar.shape[1], 1)
# Input shape for COLOR data  (number of channels, 1)
color_input_shape = (x_train_color.shape[1], 1)
# Create the model
model = create_cnn_model(lidar_input_shape, color_input_shape)

# 3. Train the model
model.summary()

history = model.fit(
    [x_train_lidar, x_train_color],  # Input data
    y_train,                         # Labels
    epochs=45,                       # Number of epochs to train for
    batch_size=32,                   # Batch size
    validation_split=0.2,  # Percentage of data to use for validation
    callbacks=[early_stopping_callback,tensorboard_callback]       # Optional callbacks, like EarlyStopping
)

# 4. Evaluate the model

loss, accuracy = model.evaluate(
    [x_test_lidar, x_test_color],
    y_test
)
print(f"Test Loss: {loss}")
print(f"Test Accuracy: {accuracy}")

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
  
