import cv2
import numpy as np
import glob
import os
from sklearn.feature_extraction import image
from sklearn.decomposition import PCA
from sklearn.externals import joblib
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
%matplotlib inline

feat_size = 10
colmap = {1: 'r', 2: 'g', 3: 'b'}

def genfeat(imgdir):
    globpatch = []
    for path in glob.glob(imgdir+"/*.jpg"):
        img = cv2.imread(path)
        # color space cvt
        img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        # patch and feature extract
        patches = image.extract_patches_2d(img, (10, 10))
        globpatch.append(patches.reshape((patches.shape[0],-1)))
    return np.vstack(globpatch)

def dim_reduction(features):
    pca = PCA(n_components=2)
    out = pca.fit_transform(features)
    joblib.dump(pca, 'pca.pkl')
    print(pca.explained_variance_)
    plt.figure()
    plt.scatter(out[:, 0], out[:,1], marker='x')
    plt.show()
    return out

def cluster(features):
    kmeans = KMeans(n_clusters=3)
    kmeans.fit(features)
    joblib.dump(kmeans, 'kmeans.pkl')
    labels = kmeans.predict(features)
    centroids = kmeans.cluster_centers_
    plt.figure()
    colors = map(lambda x: colmap[x+1], labels)
    plt.scatter(features[:, 0], features[:, 1], color=colors, alpha=0.5, edgecolor='k')
    plt.show()

if __name__ == '__main__':
    features = genfeat("images")
    newfeat = dim_reduction(features)
    cluster(newfeat)