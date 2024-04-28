import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np

mnist = tf.keras.datasets.mnist

(x_train, y_train), (x_test, y_test) = mnist.load_data()

y_train = tf.keras.utils.to_categorical(y_train)
y_test = tf.keras.utils.to_categorical(y_test)
x_train, x_test = x_train/255.0, x_test/255.0
print(x_train.shape, y_train.shape, x_test.shape, y_test.shape)

model = tf.keras.models.Sequential([
    tf.keras.layers.Reshape((28,28,1), input_shape=(28,28)),
    tf.keras.layers.Conv2D(filters=32, kernel_size=3, padding='same', activation='relu'),
    tf.keras.layers.MaxPool2D(pool_size=(2,2)),
    tf.keras.layers.Conv2D(filters=64, kernel_size=3, padding='same', activation='relu'),
    tf.keras.layers.MaxPool2D(pool_size=(2, 2)),
    tf.keras.layers.Conv2D(filters=128, kernel_size=3, padding='same', activation='relu'),
    tf.keras.layers.MaxPool2D(pool_size=(2, 2)),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(10, activation='softmax')
])
model.summary()

model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['categorical_accuracy'])
model.fit(x_train, y_train, epochs=5, validation_data=(x_test, y_test), verbose=2)
model.evaluate(x_test, y_test, verbose=2)

model.save('mnist_cnn.h5')

random_image = x_test[np.random.choice(len(x_test))]
test_image = np.expand_dims(random_image, axis = 0)

predict = model.predict(test_image)
print('predict ',predict, np.argmax(predict, axis=1))

plt.imshow(test_image[0], cmap='Greys')
plt.show()

