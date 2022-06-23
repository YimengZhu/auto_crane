import torch
import torch.nn as nn
import torch.nn.functional as F
from skimage import morphology
from torchvision import transforms
from .rcsb import Net
import cv2
import numpy as np

def mask_find_bboxs(mask):
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8) # connectivity参数的默认值为8
    stats = stats[stats[:,4].argsort()]
    return stats[:-1]


class Tester():
    def __init__(self, weight_path = "/home/kyland/hellow_py_ros/src/lxz_realsense2/src/seg/RCSB.pt",device="cuda:0"):
        self.device = torch.device(device)
        self.net = Net()
        state_dict = torch.load(weight_path, map_location=device)
        self.net.load_state_dict(state_dict)
        self.net = self.net.to(self.device)
        self.net.eval() 
        msg = "# params:{}\n".format(
            sum(map(lambda x: x.numel(), self.net.parameters())))
        print(msg)
        self.norm = transforms.Normalize(mean=(0.485, 0.458, 0.407),
                                         std=(0.229, 0.224, 0.225))


    @torch.no_grad()
    def infer(self, img):
        # img = cv2.imread(img_path)
        h, w = img.shape[:2]
        img_pixel = h * w
        min_size = img_pixel // 100
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        IMG = cv2.resize(img,(256,256))
        IMG = np.ascontiguousarray(IMG.transpose((2, 0, 1)))
        img_tensor = torch.from_numpy(IMG).float() / 255.
        x = self.norm(img_tensor).unsqueeze(0).to(self.device)
        y = self.net(x)

        pred_sal, pred_ctr = y['sal'][-1], y['ctr'][-1]
        pred_sal = F.interpolate(pred_sal, (h, w), mode='bilinear', align_corners=False)
        pred_ctr = F.interpolate(pred_ctr, (h, w), mode='bilinear', align_corners=False)

        pred_sal = torch.sigmoid(pred_sal).squeeze().detach().cpu().numpy()
        # pred_ctr = torch.sigmoid(pred_ctr).squeeze().detach().cpu().numpy()
        # pred_ctr_img = (pred_ctr * 255.).astype('uint8')
        pred_sal_img = (pred_sal * 255.).astype('uint8')
        # cv2.imshow("ctr",pred_ctr_img)
   
        new_mask = cv2.threshold(pred_sal_img, 40, 255, cv2.THRESH_BINARY)[1]
        # cv2.imshow("salasdsa",new_mask) #分割图
        _new_mask =np.array(new_mask, dtype=bool)
        dst=morphology.remove_small_objects(_new_mask,min_size=min_size,connectivity=2)  # 去除面积小于300的连通域
        newmask = (dst * 255).astype(np.uint8)
        boxes = mask_find_bboxs(newmask)
        #mask_BGR = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        new_boxes = [[int(box[0]),int(box[1]),int(box[0]+box[2]),int(box[1]+box[3])] for box in boxes]
        cv2.imwrite("seg.jpg",newmask)

        return dst, new_boxes
if __name__ == '__main__':
    T = Tester()
    img = cv2.imread("E:\kyland/realsense\seg.jpg")
    T.infer(img)
