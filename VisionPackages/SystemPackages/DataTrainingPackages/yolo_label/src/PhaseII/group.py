import glob, os

db_path = [os.path.join(os.environ["HOME"], "Desktop/image_database/yolo/PHASE_2/data/eclair"),
        os.path.join(os.environ["HOME"], "Desktop/image_database/yolo/PHASE_2/data/muffin"), 
        os.path.join(os.environ["HOME"], "Desktop/image_database/yolo/PHASE_2/data/pau")]

train_path = os.path.join(os.environ["HOME"], "catkin_ws_project/src/vision/soft_gripper_project/data_training/darknet_AB/data/phase2_train_3item.txt")
test_path = os.path.join(os.environ["HOME"], "catkin_ws_project/src/vision/soft_gripper_project/data_training/darknet_AB/data/phase2_test_3item.txt")
#if not os.path.exists(db_path):
#    os.makedirs(db_path)

# Percentage of images to be used for the test set
percentage_test = 20;

# Create and/or truncate train.txt and test.txt
file_train = open(train_path, 'w')
file_test = open(test_path, 'w')

# Populate train.txt and test.txt
#counter = 1
index_test = round(100 / percentage_test)

for path in db_path:
    for pathAndFilename in glob.iglob(os.path.join(path, "*.jpg")):
        title, ext = os.path.splitext(os.path.basename(pathAndFilename))
        num = int(title.split('_')[1])

        if num % index_test == 0:
            file_test.write(path + "/" + title + '.jpg' + "\n")
        else:
            file_train.write(path + "/" + title + '.jpg' + "\n")
            #counter = counter + 1
print "Done"
        
