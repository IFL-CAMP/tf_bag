from pathlib import Path

from tf_bag import BagTfTransformer

try:
    # ROS2
    IS_ROS2 = True
    import rclpy
    import builtin_interfaces
except ImportError:
    # ROS1
    IS_ROS2 = False
    import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion

TEST_RESOURCES_DIR = Path(__file__).resolve().parent / 'resource'
if IS_ROS2:
    TEST_BAG_PATH = TEST_RESOURCES_DIR / 'rosbagFrankaPandaRos2.bag'
else:
    TEST_BAG_PATH = TEST_RESOURCES_DIR / 'rosbagFrankaPanda.bag'


def test_open():
    bag_transformer = BagTfTransformer(TEST_BAG_PATH)


def test_get_time_at_percent():
    bag_transformer = BagTfTransformer(TEST_BAG_PATH)
    t_33 = bag_transformer.getTimeAtPercent(33)
    assert t_33 == make_time(0, 1666859840860144898)


def test_get_frame_strings():
    bag_transformer = BagTfTransformer(TEST_BAG_PATH)
    frame_strings = bag_transformer.getFrameStrings()
    assert set(frame_strings) == {'panda_link7', 'panda_rightfinger', 'panda_link2', 'panda_link4', 'panda_leftfinger',
                                  'panda_link5', 'panda_link3', 'panda_link1', 'panda_hand', 'panda_link6',
                                  'panda_link0'}


def test_get_transform_update_times():
    bag_transformer = BagTfTransformer(TEST_BAG_PATH)
    update_times = bag_transformer.getTransformUpdateTimes('world', 'panda_rightfinger', 'panda_hand',
                                                           'panda_rightfinger')
    assert list(update_times) == GET_TIMES_DATA


def test_average_transform_over_time():
    bag_transformer = BagTfTransformer(TEST_BAG_PATH)
    average_transform = bag_transformer.averageTransformOverTime('world', 'panda_rightfinger', None, None, 'panda_hand',
                                                                 'panda_rightfinger')
    if IS_ROS2:
        gt = ([0.507755105960149, 0.060171740425893884, 0.4810882680031012],
              [0.9990710582849893, -0.011146659208681233, -0.03593419415515721, 0.02101204835842193])

    else:
        gt =([0.507755105960149, 0.06017174042589391, 0.4810882680031012],
             [0.9964429667297322, -0.024410643012424973, -0.06568256081529668, 0.04681170545188851])
    assert average_transform == gt


def test_wait_and_lookup_transform():
    bag_transformer = BagTfTransformer(TEST_BAG_PATH)
    t = bag_transformer.waitForTransform('world', 'panda_rightfinger')
    tr = bag_transformer.lookupTransform('world', 'panda_rightfinger', t)
    if IS_ROS2:
        gt_stamp = builtin_interfaces.msg.Time(sec=1666859837, nanosec=363141536)
        gt_tr = Transform(
            translation=Vector3(x=0.45481430061348627, y=0.0816189178378556, z=0.643612558063392),
            rotation=Quaternion(x=0.9999999958234722, y=-5.5863857913035504e-05, z=-2.1033982849558477e-05, w=6.920878958602077e-05))
    else:
        gt_stamp = rospy.Time(1666859837, 363141537)
        gt_tr = Transform(
            translation=Vector3(x=0.4548143006134861, y=0.08161891783785571, z=0.6436125580633919),
            rotation=Quaternion(x=0.9999999958234721, y=-5.586385791314652e-05, z=-2.103398284967808e-05, w=6.920878958598185e-05))

    gt_trst = TransformStamped(header=Header(stamp=gt_stamp, frame_id='world'), child_frame_id='panda_rightfinger', transform=gt_tr)
    assert tr == gt_trst


def make_time(seconds, nanoseconds):
    if IS_ROS2:
        return rclpy.time.Time(seconds=seconds, nanoseconds=nanoseconds)
    else:
        return rospy.Time(seconds, nanoseconds)


GET_TIMES_DATA = [
    make_time(1666859837, 363141536), make_time(1666859837, 463135719), make_time(1666859837, 563133001),
    make_time(1666859837, 663151502),
    make_time(1666859837, 763144493), make_time(1666859837, 863129138), make_time(1666859837, 963135957),
    make_time(1666859838, 63136339),
    make_time(1666859838, 163132905), make_time(1666859838, 263128995), make_time(1666859838, 363132238),
    make_time(1666859838, 463132619),
    make_time(1666859838, 563144207), make_time(1666859838, 663135051), make_time(1666859838, 763137578),
    make_time(1666859838, 863139629),
    make_time(1666859838, 963134527), make_time(1666859839, 63163042), make_time(1666859839, 163169384),
    make_time(1666859839, 263168573),
    make_time(1666859839, 363155126), make_time(1666859839, 463145971), make_time(1666859839, 563476800),
    make_time(1666859839, 663184165),
    make_time(1666859839, 763143777), make_time(1666859839, 863138675), make_time(1666859839, 963141679),
    make_time(1666859840, 63193321),
    make_time(1666859840, 163147211), make_time(1666859840, 263141393), make_time(1666859840, 363136291),
    make_time(1666859840, 463139772),
    make_time(1666859840, 563192367), make_time(1666859840, 663178682), make_time(1666859840, 763144254),
    make_time(1666859840, 863132476),
    make_time(1666859840, 963143110), make_time(1666859841, 63162803), make_time(1666859841, 163146734),
    make_time(1666859841, 263141632),
    make_time(1666859841, 363130569), make_time(1666859841, 463115215), make_time(1666859841, 563142299),
    make_time(1666859841, 663130998),
    make_time(1666859841, 763186693), make_time(1666859841, 863132476), make_time(1666859841, 963140726),
    make_time(1666859842, 63124418),
    make_time(1666859842, 163145065), make_time(1666859842, 263145923), make_time(1666859842, 363139867),
    make_time(1666859842, 463121414),
    make_time(1666859842, 563145160), make_time(1666859842, 663129091), make_time(1666859842, 763128995),
    make_time(1666859842, 863136053),
    make_time(1666859842, 963151693), make_time(1666859843, 63173770), make_time(1666859843, 163155555),
    make_time(1666859843, 263151884),
    make_time(1666859843, 363157033), make_time(1666859843, 463149547), make_time(1666859843, 563152790),
    make_time(1666859843, 663149595),
    make_time(1666859843, 763119459), make_time(1666859843, 863144636), make_time(1666859843, 963146448),
    make_time(1666859844, 63144445),
    make_time(1666859844, 163150072), make_time(1666859844, 263132810), make_time(1666859844, 363146066),
    make_time(1666859844, 463153123),
    make_time(1666859844, 563141107), make_time(1666859844, 663153409), make_time(1666859844, 763154029),
    make_time(1666859844, 863143920),
    make_time(1666859844, 963142633), make_time(1666859845, 63147068), make_time(1666859845, 163155555),
    make_time(1666859845, 263133287),
    make_time(1666859845, 363141298), make_time(1666859845, 463154077), make_time(1666859845, 563143014),
    make_time(1666859845, 663141965),
    make_time(1666859845, 763133525), make_time(1666859845, 863139629), make_time(1666859845, 963148832),
    make_time(1666859846, 63168764),
    make_time(1666859846, 163154363), make_time(1666859846, 263150453), make_time(1666859846, 363373041),
    make_time(1666859846, 463140249),
    make_time(1666859846, 563139438), make_time(1666859846, 663145065), make_time(1666859846, 763212680),
    make_time(1666859846, 863139629),
    make_time(1666859846, 963140964), make_time(1666859847, 63170909), make_time(1666859847, 163158416),
    make_time(1666859847, 263392448),
    make_time(1666859847, 363139390), make_time(1666859847, 463107347), make_time(1666859847, 563112974),
    make_time(1666859847, 663135528),
    make_time(1666859847, 763146162), make_time(1666859847, 863147974), make_time(1666859847, 963143110),
    make_time(1666859848, 63143491),
    make_time(1666859848, 163133621)]
