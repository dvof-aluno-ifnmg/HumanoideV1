import torch
import torch.nn as nn
import copy

class Net(nn.Module):
    def __init__(self, input_size, hidden_layers, output_size):
        super(Net, self).__init__()
        
        layers = []
        in_features = input_size

        # Cria camadas escondidas
        for hidden_size in hidden_layers:
            layers.append(nn.Linear(in_features, hidden_size))
            layers.append(nn.ReLU())  # ativação
            in_features = hidden_size

        # Camada de saída
        layers.append(nn.Linear(in_features, output_size))
        
        # Junta tudo em uma rede sequencial
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        return self.network(x)


# Função para inicializar a rede
def init_net(inputs, hidden_layers, outputs):
    net = Net(inputs, hidden_layers, outputs)
    return net

# Função para alterar levemente os pesos
def alt_net(net, lr):
    # Faz uma cópia da rede para não sobrescrever diretamente
    new_net = copy.deepcopy(net)

    # Itera sobre todos os parâmetros (pesos e bias)
    for param in new_net.parameters():
        # Cria uma perturbação aleatória do mesmo tamanho do tensor
        noise = torch.randn_like(param) * lr
        param.data += noise
    
    return new_net