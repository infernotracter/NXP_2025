red = [255 0 0];
green = [0 255 0];
blue = [0 0 255];


ImageColor(IMG_HIGH,IMG_WIDTH,3) = uint8(0);
ImageColor(:,:,1) = Image;
ImageColor(:,:,2) = Image;
ImageColor(:,:,3) = Image;


for i = 1:IMG_HIGH
    if midline(i) > 0
        ImageColor(i,midline(i),:) = red;
    end
    if leftline(i) > 0
        ImageColor(i,leftline(i),:) = green;
    end
    if rightline(i) > 0
        ImageColor(i,rightline(i),:) = green;
    end
end

 subplot(2,2,1);
 imshow(Image);
 ImageColor(90,188,:) = blue;
 subplot(2,2,2);
imshow(ImageColor)