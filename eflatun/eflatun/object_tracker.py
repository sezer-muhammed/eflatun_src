from eflatun_msgs.msg import TrackedObjectArray
from eflatun import object_detector

def main():
    print(TrackedObjectArray().frame_seq)
    print("This is symlinked.")

if __name__ == '__main__':
    main()
