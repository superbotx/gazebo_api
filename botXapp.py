from botXsrc.botXexport import botXexport
import time
import base64, array
"""
botXexport is a dictionary containing all the reusable components you
developed for the project, and you will use them in the main program.
"""
def main():
    print('starting app ...')
    gz = botXexport['gazebo_api']['module']()
    gz.setup()

    im = gz.get_json_image()
    print("JSON Image keys: ", im[0].keys())
    print("JSON Imade encoding: ", im[0]['encoding'])
    print("JSON Image: ", bytearray(im[0]['data']))
    # time.sleep(5)
    # images = gz.get_image()
    # print("Images: ", images)

    # print (gz.get_environment_status())
    # time.sleep(5)
    # print (gz.get_environment_status())

    # Spend 2s bagging vision data (results in about 1.4s of actual bagged data)
    # gz.bag_vision(2)

    # Save an image
    # gz.get_image()

    # print("shutting down app...")
    # gz.shutdown()

"""
This is the only script that should be running from terminal so that the
program can gather modules correctly, so we need to specify main as entry point.
"""
if __name__ == '__main__':
    main()
