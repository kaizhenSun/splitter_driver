import glob
import argparse
import pandas as pd
import rosbag
import rospy
from cv_bridge import CvBridge, CvBridgeError
import os
from tqdm import tqdm
import cv2


def timestamp_float(ts):
    return ts.secs + ts.nsecs / float(1e9)


def extract_images(rosbag_path, output_path, image_topic=None,
                   start_time=None, end_time=None, zero_timestamps=False,
                   is_color=False):
    image_save_dir = f"{output_path}/rgb"
    os.makedirs(image_save_dir, exist_ok=True)

    first_ts = -1
    t0 = -1
    image_save_list = []

    with rosbag.Bag(rosbag_path, 'r') as bag:
        last_ts, img_cnt = 0, 0
        num_images = bag.get_message_count(topic_filters=image_topic)
        bar_image = tqdm(total=num_images)
        
        for topic, image_msg, t in tqdm(bag.read_messages(image_topic)):
            # if first_ts == -1:
            #     timestamp = timestamp_float(image_msg.header.stamp)
            #     first_ts = timestamp
            #     if not zero_timestamps:
            #         first_ts = -1

            # timestamp = timestamp_float(image_msg.header.stamp) - (first_ts if zero_timestamps else 0)
            timestamp = timestamp_float(t)  # Note: t is the image time of splitter!!!!! not msg.header.stamp!!!!!!
            if is_color:
                image = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")
            else:
                image = CvBridge().imgmsg_to_cv2(image_msg, "mono8")

            index = "{:0>6d}".format(int(img_cnt))
            success = cv2.imwrite(f"{image_save_dir}/{index}.png", image)

            if not success:
                print(f"image save failed: {index}.png")
            img_cnt += 1

            image_save_list.append({'frame': index,
                                    'timestamp': timestamp})
            bar_image.update(1)

        image_ts_df = pd.DataFrame(image_save_list)
        image_ts_df.to_csv(f"{image_save_dir}/ts_frame.csv", sep=' ', header=None, index=False)
    bag.close()

    return image_ts_df, first_ts

def extract_events(rosbag_path, output_path, event_topic=None,
                   image_ts_df=None, first_ts=None, zero_timestamps=False):
    event_save_dir = f"{output_path}/events"
    os.makedirs(event_save_dir, exist_ok=True)
    # limit time range
    image_ts = image_ts_df['timestamp'].values
    image_first_ts = image_ts[0]
    image_index = 0  # start from 1, because events array between two images saved in the second image index

    # if first_ts == -1 and zero_timestamps:
    #     raise ValueError("First timestamp is -1, but zero_timestamps is True")

    with rosbag.Bag(rosbag_path, 'r') as bag:
        xs, ys, ts, ps = [], [], [], []
        num_eArray = bag.get_message_count(topic_filters=event_topic)
        bar_eArray = tqdm(total=num_eArray)
        for topic, events_msg, t in tqdm(bag.read_messages(event_topic)):
            for e in events_msg.events:
                # e_ts = timestamp_float(e.ts)-(first_ts if zero_timestamps else 0)
                e_ts = timestamp_float(e.ts)
                # # Don't save events before the first image
                # if e_ts < float(image_first_ts):
                #     continue
                # if len(xs) % 1000 == 0:
                # print(f"events ts {e_ts}")
                # print(f"limit:{float(image_ts[image_index])}")
                xs.append(e.x)
                ys.append(e.y)
                ts.append(e_ts)
                ps.append(1 if e.polarity else -1)
                # Judge whether the timestamp of the event is in the range of the image
                if e_ts > float(image_ts[image_index]):
                    e_array_df = pd.DataFrame({'t': ts, 'x': xs, 'y': ys, 'p': ps})
                    event_name = "{:0>6d}".format(int(image_index))
                    e_array_df.to_csv(f"{event_save_dir}/{event_name}.txt", sep=' ', index=False, header=False)
                    # print(f"save {event_name}.txt !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    # print(f"length of df: {e_array_df.shape[0]}")
                    del xs[:]
                    del ys[:]
                    del ts[:]
                    del ps[:]

                    if image_index < len(image_ts)-1:
                        image_index += 1
                    else:
                        break
            bar_eArray.update(1)
    bag.close()


def extract_rosbags(rosbag_paths, output_dir,
                    event_topic, image_topic,
                    zero_timestamps=False):
    for path in rosbag_paths:
        bagname = os.path.splitext(os.path.basename(path))[0]
        bag_out_dir = os.path.join(output_dir, "{}".format(bagname))
        os.makedirs(bag_out_dir, exist_ok=True)
        print("Extracting {} to {}".format(path, bag_out_dir))
        # Extract Images first
        image_ts_df, first_ts = extract_images(path, bag_out_dir, image_topic=image_topic, is_color=True,
                                               zero_timestamps=False)
        print(f"first ts: {first_ts}")
        # Extract Events
        extract_events(path, bag_out_dir, event_topic=event_topic,
                       image_ts_df=image_ts_df, first_ts=first_ts, zero_timestamps=False)


if __name__ == "__main__":
    """
    Tool for converting rosbag events to an efficient HDF5 format that can be speedily
    accessed by python code.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--path",
                        default="/home/jhang/workspace/Dataset_tools/splitter_driver/src/Datasets/raw_bag",
                        help="ROS bag file to extract or directory containing bags")
    parser.add_argument("--output_dir",
                        default=None,
                        help="Folder where to extract the data")
    parser.add_argument("--event_topic",
                        default='/capture_node/events',
                        help="Event topic")
    parser.add_argument("--image_topic",
                        default='/image_raw',
                        help="Image topic (if left empty, no images will be collected)")
    args = parser.parse_args()

    if args.output_dir is None:
        args.output_dir = f"{os.path.dirname(args.path)}/extracted"
        os.makedirs(args.output_dir, exist_ok=True)

    print('Data will be extracted in folder: {}'.format(args.output_dir))
    if os.path.isdir(args.path):
        rosbag_paths = sorted(glob.glob(os.path.join(args.path, "*.bag")))
    else:
        rosbag_paths = [args.path]
    print('Found {} rosbags'.format(len(rosbag_paths)))
    extract_rosbags(rosbag_paths, args.output_dir, args.event_topic, args.image_topic)
