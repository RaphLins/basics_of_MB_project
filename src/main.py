from myThymio import MyThymio

TIME_STEP = 0.1

def main():
    # myThymio = MyThymio(port="/dev/cu.usbmodem14401", robot_path=robot_path, refreshing_rate=2 * TIME_STEP)
    myThymio = MyThymio(port="COM3", refreshing_rate=2 * TIME_STEP)
    myThymio.run()


if __name__ == "__main__":
    main()
