import os
from PIL import Image
data_dir = "./data/230613_remocon"
png_dir = os.path.join(data_dir, "png/")
save_dir = os.path.join(data_dir, "png1/")

def crop_center(pil_img, crop_width, crop_height):
    img_width, img_height = pil_img.size
    return pil_img.crop(((img_width - crop_width) // 2,
                         (img_height - crop_height) // 2,
                         (img_width + crop_width) // 2,
                         (img_height + crop_height) // 2))

for f in os.listdir(png_dir):
    if f.endswith(".JPG"):
        
        photo = Image.open(os.path.join(png_dir, f))
        print(photo)
        photo_resize = photo.resize((1440,1920), resample=Image.LANCZOS)
        photo_resize = crop_center(photo_resize, 1080, 1920)
        photo_resize.save(os.path.join(save_dir, f))
        print("ok")