import torch
import torch.optim as optim
from torchvision import transforms

from Dataset import Fruit_Dataset
from NN import CNN

NUM_EPOCHS = 100
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
    checkpoint = torch.load(MODEL_PATH)
    net.load_state_dict(checkpoint['model_state_dict'])

    total = 0
    correct = 0
    for i, data in enumerate(testloader, 0):

        inputs = data[0].to(device=device, dtype=torch.float)
        inputs = inputs.permute((0, 3, 1, 2))
        labels = data[1].to(device=device, dtype=torch.long)

        outputs = net(inputs)


        for i in range(len(outputs)):
            if torch.argmax(outputs[i]) == labels[i]:
                correct += 1
            total += 1

    print ("Accuracy: " + str(float(correct)/float(total)))


if __name__ == "__main__":
    transform_data = transforms.Compose([
        transforms.RandomHorizontalFlip(),
        # transforms.RandomVerticalFlip(),
        #transforms.RandomResizedCrop(556, scale=(0.7, 1.0), ratio=(0.75, 1.3333333333333333), interpolation=2),
        # transforms.ColorJitter(brightness=0.1, contrast=0.1, saturation = 0.1),
        transforms.ToTensor(),
    ])

    net = CNN(3)
    
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print("Training device: ", device)
    net.to(device)

    # train_data = Fruit_Dataset(train = True, transform = transform_data)
    # trainloader = torch.utils.data.DataLoader(train_data, batch_size = BATCH_SIZE, shuffle = True, num_workers = 1)
    # train(net, trainloader, device)

    test_data = Fruit_Dataset(train = False, transform = None)
    testloader = torch.utils.data.DataLoader(test_data, batch_size = BATCH_SIZE, shuffle = False, num_workers = 1)
    test(net, testloader, device)