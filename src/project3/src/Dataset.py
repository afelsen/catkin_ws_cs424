import cv2
import numpy as np
from torch.utils.data import Dataset
import glob
from PIL import Image
import random 

class Fruit_Dataset(Dataset):
    def __init__(self, train=False, transform = None):
        self.transform = transform
        self.data = []

        data_dict = {}
        self.classes = ["watermelon", "grapes", "pineapple"]
        self.labels = [0, 1, 2]
        for name, label in zip(self.classes, self.labels):
            data_dict[name] = []
            for file in glob.glob("./data/" + name +"/*.jpg"):
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
        if self.transform:
            img = Image.fromarray(img)

            img = self.transform(img)
            img = img.permute(1,2,0)

            img = np.array(img)
        else:
            pass
            # img = img[..., np.newaxis]

        return img, self.data[idx][1]
    
    def __len__(self):
        return len(self.data)
