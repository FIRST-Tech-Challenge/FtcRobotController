# the output of this file goes into the file located here C:\Users\BC4HSTEM\.eocvsim\eocvsim_sources
import json
import os


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


def create_test_ocvsim_file(folder_path, out_file_name):

    image_sources = {}

    directory = os.fsencode(folder_path)

    for file in os.listdir(directory):
        filename = os.fsdecode(file)
        if filename.endswith(".jpg"):
            # print(os.path.join(directory, filename))
            filename_no_ext = os.path.basename(filename).split('.')[0]
            image_sources[filename_no_ext] = {
                "imgPath": folder_path + '/' + filename,
                "size": {
                    "width": 240.0,
                    "height": 240.0
                },
                "createdOn": 1642189527786
            }
            continue
        else:
            continue

    eocv_json = {
        "imageSources": image_sources,
        "cameraSources": {},
        "videoSources": {}
    }

    with open(out_file_name, "w") as outfile:
        json.dump(eocv_json, outfile)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    create_test_ocvsim_file('C:/Users/BC4HSTEM/OpenCV/01_12_2022', 'C:/Users/BC4HSTEM/OpenCV/01_12_2022/eocvdata.json')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
