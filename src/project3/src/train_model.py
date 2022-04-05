import torch
import torch.optim as optim
from torchvision import transforms
from torch.nn import Softmax

from Dataset import Fruit_Dataset
from NN import CNN

NUM_EPOCHS = 500
MODEL_PATH = "./models/model.py"
BATCH_SIZE = 4

def train(net, trainloader, device):

    criterion = torch.nn.CrossEntropyLoss()
    optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

    for epoch in range(NUM_EPOCHS):

        running_loss = 0.0
        for i, data in enumerate(trainloader, 0):

            inputs = data[0].to(device=device, dtype=torch.float)
            inputs = inputs.permute((0, 3, 1, 2))
            labels = data[1].to(device=device, dtype=torch.long)


            optimizer.zero_grad()
            outputs = net(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            running_loss += loss.item()
            optimizer.step()

            print ("Loss:" + str(running_loss/(i+1)) + "Epoch:" + str(epoch) + "\r"),
        print()


    torch.save({
        "model_state_dict": net.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "epoch": NUM_EPOCHS
    }, MODEL_PATH)

    print('Finished Training')

def test(net, testloader, device):
    checkpoint = torch.load(MODEL_PATH, map_location=torch.device('cpu'))
    net.load_state_dict(checkpoint['model_state_dict'])

    total = 0
    correct = 0
    for i, data in enumerate(testloader, 0):

        inputs = data[0].to(device=device, dtype=torch.float)
        inputs = inputs.permute((0, 3, 1, 2))
        labels = data[1].to(device=device, dtype=torch.long)

        outputs = net(inputs)

        print(outputs)
        
        for i in range(len(outputs)):
            if torch.argmax(outputs[i]) == 0 and labels[i] == 0:
                correct += 1
            elif torch.argmax(outputs[i]) != 0 and labels[i] != 0:
                correct += 1
            total += 1

    print ("Accuracy: " + str(float(correct)/float(total)))


if __name__ == "__main__":
    transform_watermelon = transforms.Compose([
        transforms.RandomHorizontalFlip(),
        transforms.RandomResizedCrop(64, scale=(0.08, 1.0), ratio=(0.5, 1.5), interpolation=2),
        transforms.ColorJitter(brightness=.5, contrast=.5, saturation = .5),
        # transforms.RandomInvert(),
        transforms.RandomPerspective(distortion_scale=0.6, p=1.0),
        transforms.ToTensor(),
    ])

    transform_others = transforms.Compose([
        transforms.RandomHorizontalFlip(),
        transforms.RandomVerticalFlip(),
        transforms.RandomResizedCrop(64, scale=(0.08, 1.0), ratio=(0.2, 2), interpolation=2),
        transforms.ColorJitter(brightness=1, contrast=1, saturation = 1),
        # transforms.RandomInvert(),
        transforms.RandomPerspective(distortion_scale=0.6, p=.5),
        transforms.ToTensor(),
    ])

    net = CNN(2)
    
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print("Training device: ", device)
    net.to(device)

    train_data = Fruit_Dataset(train = True, watermelon_transform=transform_watermelon, other_transform=transform_others)
    trainloader = torch.utils.data.DataLoader(train_data, batch_size = BATCH_SIZE, shuffle = True, num_workers = 1)
    train(net, trainloader, device)

    test_data = Fruit_Dataset(train = False)
    testloader = torch.utils.data.DataLoader(test_data, batch_size = BATCH_SIZE, shuffle = False, num_workers = 1)
    test(net, testloader, device)