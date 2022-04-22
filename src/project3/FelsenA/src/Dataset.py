import cv2
import numpy as np
from torch.utils.data import Dataset
import glob
from PIL import Image
import random 

class Fruit_Dataset(Dataset):
    def __init__(self, train=False, watermelon_transform = None, other_transform = None):
        self.watermelon_transform = watermelon_transform
        self.other_transform = other_transform
        self.data = []

        data_dict = {}
        self.classes = ["watermelon", "grapes", "pineapple"]
        self.labels = [0, 1, 1]
        for name, label in zip(self.classes, self.labels):
            data_dict[name] = []
            for file in glob.glob("./data/" + name +"/*.*"):
                image = cv2.imread(file)
                image = cv2.resize(image, (64,64))
                
                data_dict[name].append((image, label))

                print (str(file) + "\r      "),
        print()

        for k, data in data_dict.items():
            random.seed(192093)
            random.shuffle(data)
            if train:
                self.data += data[:int(.8*len(data))]
            else:
                self.data += data[int(.8*len(data)):]
    
    def __getitem__(self, idx):
        img = self.data[idx][0]
        if self.watermelon_transform and self.other_transform:
            
            img = Image.fromarray(img)

            if self.data[idx][1] == 0:
                img = self.watermelon_transform(img)
            else:
                img = self.other_transform(img)
            img = img.permute(1,2,0)

            img = np.array(img)


            # cv2.imshow("test", img)
            # cv2.waitKey(0)
        else:
            pass
            # img = img[..., np.newaxis]

        return img, self.data[idx][1]
    
    def __len__(self):
        return len(self.data)
