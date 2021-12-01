import tensorflow as tf

optimized_graph_path = "C:/development/BC4HStem/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_tse/converted_freight_frenzy_tse_optimized/freight_frenzy_tse_optimized_graph.pb"
output_pbtxt = "C:/development/BC4HStem/FtcRobotController/TeamCode/src/main/assets/tf_models/freight_frenzy_tse/converted_freight_frenzy_tse_optimized/freight_frenzy_tse_optimized_graph.pbtxt"

# Read the graph.
with tf.gfile.FastGFile(optimized_graph_path, "rb") as f:
    graph_def = tf.GraphDef()
    graph_def.ParseFromString(f.read())
# Remove Const nodes.
for i in reversed(range(len(graph_def.node))):
    if graph_def.node[i].op == 'Const':
        del graph_def.node[i]
    for attr in ['T', 'data_format', 'Tshape', 'N', 'Tidx', 'Tdim',
                 'use_cudnn_on_gpu', 'Index', 'Tperm', 'is_training',
                 'Tpaddings']:
        if attr in graph_def.node[i].attr:
            del graph_def.node[i].attr[attr]
# Save as text.
tf.train.write_graph(graph_def, "", output_pbtxt, as_text=True)