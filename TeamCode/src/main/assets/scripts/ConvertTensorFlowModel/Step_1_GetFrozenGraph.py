import tensorflow as tf
from tensorflow import keras
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2
import numpy as np
from keras.models import load_model

#path of the directory where you want to save your model
frozen_out_path = '/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/tse_giant_sensor_converted_keras/frozen'

# name of the .pb file
frozen_graph_filename = 'freight_frenzy_barcodes_graph'

model = load_model('/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/tse_giant_sensor_converted_keras/keras_model.h5')
#model = tf.saved_model.load('/Users/alex/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_barcodes/tse_giant_sensor_converted_savedmodel/savedmodel/saved_model.pb')

# Convert Keras model to ConcreteFunction
full_model = tf.function(lambda x: model(x))
full_model = full_model.get_concrete_function(
    tf.TensorSpec(model.inputs[0].shape, model.inputs[0].dtype))

# Get frozen graph def
frozen_func = convert_variables_to_constants_v2(full_model)
graph_def = frozen_func.graph.as_graph_def()

# Remove NoOp nodes
for i in reversed(range(len(graph_def.node))):
    if graph_def.node[i].op == 'NoOp':
        del graph_def.node[i]

for node in graph_def.node:
    for i in reversed(range(len(node.input))):
        if node.input[i][0] == '^':
            del node.input[i]

layers = [op.name for op in frozen_func.graph.get_operations()]
print("-" * 60)
print("Frozen model layers: ")
for layer in layers:
    print(layer)
print("-" * 60)
print("Frozen model inputs: ")
print(frozen_func.inputs)
print("Frozen model outputs: ")
print(frozen_func.outputs)

tf.io.write_graph(graph_or_graph_def=graph_def,
                  logdir=frozen_out_path,
                  name=f"{frozen_graph_filename}.pb",
                  as_text=False)
tf.io.write_graph(graph_or_graph_def=graph_def,
                  logdir=frozen_out_path,
                  name=f"{frozen_graph_filename}.pbtxt",
                  as_text=True)

