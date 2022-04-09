import tensorflow as tf

print('TF Devices:', tf.config.list_physical_devices())
print('TF Device Placement:', tf.config.get_soft_device_placement())

tf.debugging.set_log_device_placement(True)