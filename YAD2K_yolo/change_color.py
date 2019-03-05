import cv2
import os
import numpy as np
import glob
import shutil

def change_light(img, L=1.7, S=1.4):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    img=img.astype(np.float32)
    img[:,:,1] *= L
    img[:, :, 2] *= S
    img[img>255] = 255
    img = img.astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_HLS2BGR)
    return img

def change_light_from_path(imgroot, newroot=None, L=1.7, S=1.4):
    if newroot is None:
        newroot = imgroot+'_changed'
    if not os.path.exists(newroot):
        os.mkdir(newroot)
    for imgpath in glob.glob(imgroot+'/*.jpg'):
        img = cv2.imread(imgpath)
        changed = change_light(img)
        new_path = newroot+'/'+os.path.basename(imgpath)
        cv2.imwrite(new_path, changed)


def color_balance(img):
    b, g, r = cv2.split(img)
    B = np.mean(b)
    G = np.mean(g)
    R = np.mean(r)
    K = (R + G + B) / 3
    Kb = K / B
    Kg = K / G
    Kr = K / R
    cv2.addWeighted(b, Kb, 0, 0, 0, b)
    cv2.addWeighted(g, Kg, 0, 0, 0, g)
    cv2.addWeighted(r, Kr, 0, 0, 0, r)
    merged = cv2.merge([b,g,r])
    return merged

def color_balance_from_path(img_path, dest_root):
    img = cv2.imread(img_path)
    img = color_balance(img)
    base_name = os.path.basename(img_path)
    base_name, ext = os.path.splitext(base_name)
    new_path = "%s/%s_adj%s" % (dest_root, base_name, ext)
    print(new_path)
    cv2.imwrite(new_path, img)
    # cv2.imshow("a", merged)
    # cv2.waitKey(0)

def color_balance_from_root(src_root, dest_root):
    if not os.path.exists(dest_root):
        os.makedirs(dest_root)
    for img_path in glob.glob(src_root + "/*.jpg"):
        color_balance_from_path(img_path, dest_root)

def move_txts(srcroot, dest_root, suffix='_adj'):
    if not os.path.exists(dest_root):
        os.makedirs(dest_root)
    for txt in glob.glob(srcroot+'/*.txt'):
        filename = os.path.basename(txt)
        newpath = dest_root + '/' + filename.replace('.txt', suffix+'.txt')
        shutil.copyfile(txt, newpath)


if __name__ == '__main__':
    # change_color_from_path(r"D:\project\cam_save\imgs\original\route3_meet2")
    src_root = r"D:\project\cam_save\imgs\26_img_mixed"
    # src_root = r"D:\project\cam_save\imgs\original_mixed"
    dest_root =  src_root+'_adj'
    # color_balance_from_root(src_root, dest_root)
    # change_light_from_path(dest_root, dest_root, L=1.3, S=1)
    move_txts(src_root, dest_root)





