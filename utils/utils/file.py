# coding=utf-8

import os
import csv
from collections import OrderedDict
import pandas as pd
import numpy as np
from PIL import Image


def mkdir(*path):
    if path is None:
        raise LookupError('Set a path')
    if os.path.isdir(os.path.join(*path)):
        print('%s already exists.' % (pathname(*path)))
    else:
        os.makedirs(os.path.join(*path))
    return pathname(*path)


def is_file(filename):
    if os.path.isfile(filename):
        return True
    return False


def folder_exists(path):
    if os.path.isdir(path):
        return True
    return False


def delete_file(filename):
    if not is_file(filename):
        return
    else:
        os.remove(filename)

def get_folders(path=None,key=None):
    if path:
        return filter(lambda f: os.path.isdir(f),  map(lambda f: os.path.join(path,f),sorted(os.listdir(path),key = key)))
    return None

def get_files(path,key=None):
    return [pathname(path,file) for file in sorted(os.listdir(path),key=key) if is_file(pathname(path,file))]

def pathname(*path,**kwargs):
    if kwargs and kwargs['flag'] == 1:
        if not folder_exists(os.path.abspath(os.path.join(*path))):
            return mkdir(os.path.abspath(os.path.join(*path)))

    return os.path.abspath(os.path.join(*path))


class Csv:
    def __init__(self,filename, headers=None, mode=None):
        if filename is None:
            raise LookupError('Missing parameters filename')
        self.filename= filename

        if headers is not None:
            self.headers = headers
            self.mode = mode
        else:
            self.mode ='r'

        if mode == 'w' or mode == 'a':
            if not is_file(self.filename):
                with open(self.filename,'w') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(self.headers)

    def write(self, rows):
        if rows is None is None:
            raise LookupError('Missing parameters')
        with open(self.filename, self.mode) as csvfile:
            writer = csv.writer(csvfile)

            for row in list(rows):
                writer.writerow(row)

    def read(self):
        data = OrderedDict()
        with  open(self.filename, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            headers = next(csvreader)
            for header in headers:
                data[header] = list()
            for row in csvreader:
                for i in range(len(row)):
                    data[data.keys()[i]].append(row[i])

        return data

class Pd:
    def __init__(self,filename=None,columns= None,shape=None):
        # if filename is None:
        #     raise LookupError('Missing parameters filename')
        self.filename = filename
        # if not file_exists(self.filename):
            
        self.columns = columns
        self.array = np.empty(shape = shape)

    def write_csv(self):
        df = pd.DataFrame(data = self.array,columns = self.columns)
        df.to_csv(self.filename,columns = self.columns)

    def write_npy(self):
        np.save(self.filename,self.array)

    def read(self):
        df = pd.read_csv(self.filename,index_col=0)
        return df

    def concatenate(self,array,axis=0):
        self.array = np.concatenate((self.array,array),axis = axis)


class Img:

    def __init__(self):
        pass

    def img_from_array(self, array, save_path= None):
        img = Image.fromarray(array)
        if save_path:
            img.save(fp = save_path)
        return img
