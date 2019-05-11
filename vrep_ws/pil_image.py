from PIL import Image
import numpy as np

i =0
while True:
	load_array = '/media/mtb/Data Disk/JACO/DEMO/STACKING/DEMO_0/img/img_%i.npy'%i
	image = Image.fromarray(np.load(load_array), 'RGB')
	image.save('/media/mtb/Data Disk/JACO/DEMO/STACKING/DEMO_0/img/img_%i.png'%i)
	i +=1