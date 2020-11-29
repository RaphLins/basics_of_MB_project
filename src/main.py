from myThymio import MyThymio


TIME_STEP = 0.1


def main():
    robot_path = [(0, 0, 0), (210, 0, 0), (210, 148.5, 0), (0, 148.5, 0)]
    myThymio = MyThymio(port="/dev/cu.usbmodem14401", robot_path=robot_path, refreshing_rate=2 * TIME_STEP)
    myThymio.run()


if __name__ == "__main__":
    main()
