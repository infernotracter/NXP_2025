%�Ӵ��ڷ��ص�����ͼ����
clc;
clear;

%��ȡͼ��
cameraReceiver = imread('188x120.bmp');
[IMG_BUF_HIGH,IMG_BUF_WIDTH] = size(cameraReceiver);%������188x120
IMG_HIGH = 56; IMG_WIDTH = 93 ; %ʵ��ʹ��94x56

%ת����C�е�ַ����ʽ
fullCameraBufferAddr(IMG_BUF_HIGH*IMG_BUF_WIDTH) = uint8(0);
Image(56,94) = uint8(0);

%fullCameraBufferAddrΪC��������ʼ��ַ
for i = 1:120
	fullCameraBufferAddr(188*(i-1)+1:188*i) = cameraReceiver(i,:);
end


for i = 2:IMG_HIGH
    for j = 2:IMG_WIDTH

        x = 2*188*i;
        y = 2*j;
        Image(i, j) =     ...
        fullCameraBufferAddr( 2*188*(i-1) + 2*(j-1) )/9 + fullCameraBufferAddr( 2*188*(i-1)  + 2*j )/9 + fullCameraBufferAddr( 2*188*(i-1) + 2*(j+1) )/9 +...
        fullCameraBufferAddr( 2*188*(i)    + 2*(j-1) )/9 + fullCameraBufferAddr( 2*188*(i+1)     + 2*(j+1) )/9 + fullCameraBufferAddr( 2*188*(i)     + 2*(j+1) )/9 + ...
        fullCameraBufferAddr( 2*188*(i+1)+ 2*(j-1) )/9 + fullCameraBufferAddr( 2*188*(i+1) + 2*j )/9 + fullCameraBufferAddr( 2*188*(i+1) + 2*(j+1) )/9;
        
        
    end
end



%��ʾԭͼ
subplot(2,2,1);
imshow(cameraReceiver);
title('ԭͼ');

subplot(2,2,2);
imshow(Image);
title('ԭͼ');