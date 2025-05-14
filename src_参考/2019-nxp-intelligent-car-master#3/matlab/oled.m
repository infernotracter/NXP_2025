%�Ӵ��ڷ��ص�����ͼ����
clc;
clear;

%��ȡͼ��
cameraReceiver = imread('188x120.bmp');
[IMG_BUF_HIGH,IMG_BUF_WIDTH] = size(cameraReceiver);%����ͷ�ߴ�



%��HEX����
bin_image = textread('����1.txt','%c');

%ѭ���߽�ֵѡ��
image_data_length = size(bin_image);
IMG_HIGH    = 56;
IMG_WIDTH  = 94;
frame_size = IMG_HIGH*IMG_WIDTH/8;

%����һ��RGBͼƬ
R=1;G=2;B=3;
Image(IMG_HIGH,IMG_WIDTH,3) = uint8(0);

%��������    dec2hex(image(1))    dec2hex(abs(a(1)))
ImageByteCount = 1;

%֡ͷ�ж� 0x55 0x55
if hex2dec('55') == bin_image(ImageByteCount) && hex2dec('55') == bin_image(ImageByteCount+1)
    ImageByteCount = ImageByteCount + 2; %����֡ͷ
    for high = 0:6
        for j = 1:94        %��������
            for i = 1:8         %ת��һ���ֽ�
                PixleTemp = bitget(abs(bin_image(ImageByteCount)),i) * 255;
                Image(i+high*8,j,:) = [PixleTemp, PixleTemp, PixleTemp];
            end
            ImageByteCount = ImageByteCount + 1;
        end
    end
end

%Image = imread('ojbk.bmp');
subplot(1,2,1);
imshow(Image);
title('ԭʼͼ��');

 


%/* ����������Ϣ */
midline(IMG_HIGH)   = int16(0);
leftline(IMG_HIGH)    = int16(0);
rightline(IMG_HIGH) = int16(0);

mid = 46;
for i = IMG_HIGH:-1:1
	leftline(i) = 1;
    for j = mid:-1:1       %�������
            if Image(i,j) == 0
                leftline(i) = j;
                break
            end
    end

    rightline(i) = IMG_WIDTH;
	for j = mid:IMG_WIDTH       %���ұ���
         if Image(i,j)==0
            rightline(i) = j;
            break
         end
	end  

    mid = uint8((leftline(i)+rightline(i))/2);

    if leftline(i) == rightline(i)
        midline(i) = -1;    %%��Ч�������Ѿ�Ѱ�ҽ���  ���ߺͱ��߽���
    else
        midline(i) = mid;       
    end

    if midline(i) >0
        Image(i,midline(i),R) = 255; %����ԭ����
        Image(i,midline(i),G) = 0;
        Image(i,midline(i),B) = 0; 
    end
end

%����У���㷨

%�������������ߺͱ��߽���
CrossoverPointY = 1;
for i = 35:-1:1
    if midline(i) == -1
        CrossoverPointY = i;
        break;
    end
end
%�ӽ�����������������߽罻��

if CrossoverPointY ~=1 %���ߺ������н��㣬ת�䲻��
    if midline(CrossoverPointY+1) < 46
        p1 = leftline;
        p2 = rightline;
        BoundValue = 1; 
    else
        p1 = rightline;
        p2 = leftline;  
        BoundValue = IMG_WIDTH;
    end

    for i = CrossoverPointY+1:35
        if p1(i) ~= BoundValue
            RoadLowBound = i;
            break;
        end
    end

    shift = p2(RoadLowBound) - midline(RoadLowBound);

    %�ñ��ߴ�������
    for i = CrossoverPointY:RoadLowBound
        midline(i) = p2(i) - shift;
    end
    midline(CrossoverPointY+1) = BoundValue;

end


%�������ߵ�һ���˲�

aaaaa = 0.7;
for i = CrossoverPointY+1:IMG_HIGH-1
   midline(i) = aaaaa*midline(i) + (1-aaaaa)*midline(i+1);
   
    if midline(i)>0
        Image(i,midline(i),R) = 0; %��������
        Image(i,midline(i),G) = 0;
        Image(i,midline(i),B) = 255;
    end
end


%OELD������������
for i = 1:IMG_HIGH
    Image(i,leftline(i),R) = 0; %���������
    Image(i,leftline(i),G) = 255;
    Image(i,leftline(i),B) = 0;
    Image(i,rightline(i),R) = 0; %����you����
    Image(i,rightline(i),G) = 255;
    Image(i,rightline(i),B) = 0;
end
subplot(1,2,2);
imshow(Image);
title('������ȡ');