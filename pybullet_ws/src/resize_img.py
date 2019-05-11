
from utils import get_folders , mkdir
import os
import numpy as np
from skimage.transform import resize
import skimage as sk

# PATH = '/home/barry/hdd/IMITATION/training_data/STACK'
#
# folders = sorted(get_folders(PATH),key = lambda file: int(file.split('/')[-1][6:]))
#
# for f in folders[1:]:
#     img_folder = os.path.join(f,'IMG_64x64')
#
#     mkdir(img_folder)
#     img = np.load(os.path.join(f,'VELOCITIES','img.npy')).reshape(-1,128,128,4)[1:][:,:,:,:3]
#
#     for i in range(img.shape[0]):
#         rgb = resize(img[i], (64, 64,3), anti_aliasing = True)
#         np.save(os.path.join(img_folder,'img_%d'%i),rgb)
#     print f


# for f in folders[0:1]:
#     img_folder = os.path.join(f, 'IMG')
#
#     for im in os.listdir(img_folder):
#         rgb = np.load(os.path.join(img_folder,str(im)))
#         jpg = os.path.join(img_folder,str(im)[:-4]+'.jpg')
#         scipy.misc.imsave(jpg, rgb)



    # for i in range(img.shape[0]):
    #     rgb = resize(img[i], (64, 64,3), anti_aliasing = True)
    #     np.save(os.path.join(img_folder,'img_%d'%i),rgb)


#
PATH = '/home/barry/hdd/IMITATION/training_data/THROW'

folders = sorted(get_folders(PATH),key = lambda file: int(file.split('/')[-1][6:]))

for f in folders[0:]:

    img_folder = os.path.join(f,'IMG_128x128')
    mkdir(img_folder)
    img = np.load(os.path.join(f,'img.npy')).reshape(-1,128,128,4)[1:][:,:,:,:3]
    for i in range(img.shape[0]):
        # rgb = resize(img[i], (64, 64,3), anti_aliasing = True)
        rgb = img[i]
        np.save(os.path.join(img_folder,'img_%d'%i),rgb)
    print f