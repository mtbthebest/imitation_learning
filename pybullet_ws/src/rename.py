
from utils import get_folders,pathname
import os
# PATH = '/home/barry/hdd/IMITATION/training_data/STACK'
PATH = '/data/barry/IMITATION/dataset/THROW/DEMO'

# folders = get_folders(PATH, key=lambda f : int(f[6:]))[1000:]
folders = get_folders(PATH, key=lambda f : int(f[5:]))
# print(len(folders))

# rg=0
b = range(len(folders))

for fold in folders:
    # os.rename(fold,pathname(PATH,'demo_%d'%rg))
    # rg +=1
    a = fold.split('/')[-1][5:]
    if int(a) not in b:
        print fold

# if rename:
#     for i,fold in enumerate(sorted(folders,key = lambda file: int(file.split('/')[-1][6:]))):
#         f = int(fold.split('/')[-1][6:])
#
#         os.rename(os.path.join(PATH,'Trial_%d'%f ),
#                   os.path.join(PATH, 'Trial_%d' % i))


