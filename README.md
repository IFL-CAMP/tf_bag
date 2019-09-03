# tf_bag


Utilities to transparently use tf data recorded with rosbag

Querying tf for an arbitrary transformation is very comfortable
at runtime thanks to its tooling (`tf_echo` from the console and
the `TransformListener` programmatically). The programs included in the
tf package implement a background recording of the messages incoming on
the `/tf` topic and assemblying them in a Direct Acyclic Graph, which
can then be looked up between two arbitrary nodes.

While it is possible to include the `/tf` and `/tf_static` topics to the 
ones recorded by rosbag, no tool is provided to use this data. So the 
most common solution is to play the rosbag and let a program poll tf regularly. 
This is not an ideal solution, especially for scripting.

This package includes a `BagTfTransformer` which is able to use tf data
from a recorded rosbag by feeding a tf `TransformerROS`.
It supports looking up a transform at a given time,
waiting for a transform since a specific time, and much more. The API was
thought to be as similar as possible as the tf classes. The performance
was optimized for scripting purposes (e.g. linear scans over time).

## Common tasks

#### Recording data into a bag file
```bash
# save to a custom location, compress on the fly
rosbag record -O PATH_TO_MY_BAG/data.bag --lz4 /tf /tf_static MY_TOPIC1 MY_TOPIC2 <...>
```

#### Loading data from a bag file
```python
import rosbag
from tf_bag import BagTfTransformer

bag_file_path = '/path/to/some.bag'

bag = rosbag.Bag(bag_file_path)

bag_transformer = BagTfTransformer(bag)
```

Or alternatively:

```python
from tf_bag import BagTfTransformer

bag_transformer = BagTfTransformer('/path/to/some.bag')
```

#### Displaying the transforms included in a bag
```python
print(bag_transformer.getTransformGraphInfo())
```

#### Looking up a transform
```python
translation, quaternion = bag_transformer.lookupTransform(frame1_id, frame2_id, time)
```

The transformer takes care of "waiting" for the transform for up to 0.1
seconds.

#### Waiting for a transform
```python
first_transform_time = bag_transformer.waitForTransform(frame1_id, frame2_id, start_time)
```

The start_time parameter can be omitted: in that case, the transformer will
start waiting from the beginning of the bag data.

#### Processing a transform

In order to process a transform, it is necessary to specify the times at
which it should be sampled. If the two frames are directly connected, the
transform will be sampled at every update (that is, at the timestamp of
which message on the tf topic having the source frame as the header.stamp.frame_id
attribute and the target frame as the child_frame_id attribute). If the
two frames are not directly connected, an alternate "trigger" source or target
frame (or both) must be specified.

```python
# the two frames are directly connected in the tf tree
average_translation, average_quaternion = bag_transformer.averageTransform(frame1_id, frame2_id)

 # the transform will be sampled at every update of the transform between frame1 and frame2
average_translation, average_quaternion = bag_transformer.averageTransform(frame3, frame6,
                                                                           trigger_orig_frame=frame1_id,
                                                                           trigger_dest_frame=frame2_id)
```

For particular needs, a callback can be provided:
```python
translation_z_over_time = bag_transformer.processTransform(lambda time, transform: transform[0][2], 
                                                           frame1_id, frame2_id, start_time)
```

#### Visualization

The translation of a transform can be visualized in a matplotlib graph.
If no axis is specified, a 3D plot will be drawn:
```python
bag_transformer.plotTranslation(frame1, frame2)
```

Otherwise, the value of the translation in one axis will be plotted over time:
```python
bag_transformer.plotTranslation(frame1, frame2, axis='z')
```

