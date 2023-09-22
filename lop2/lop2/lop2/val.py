
import cv2
import math
import numpy as np



def normalize (img, img_mean, img_scale):

    img = np.array(img, dtype=np.float32)
    img = (img - img_mean) * img_scale

    return img


def pad_width (image, stride, pad_value, min_dims):

    image_height, image_width, _ = image.shape
    image_height = min(min_dims[0], image_height)

    min_dims = (
        math.ceil(min_dims[0] / stride) * stride,
        math.ceil(max(min_dims[1], image_width) / stride) * stride
    )

    a = int(math.floor( (min_dims[0] - image_height) / 2.0) )
    b = int(math.floor( (min_dims[1] - image_width)  / 2.0) )

    padding = (
        a,
        b,
        int(min_dims[0] - image_height - a),
        int(min_dims[1] - image_width - b)
    )

    padded_img = cv2.copyMakeBorder(src=image, top=padding[0], bottom=padding[2], left=padding[1], right=padding[3],
                                    borderType=cv2.BORDER_CONSTANT, value=pad_value)
    return padded_img, padding
