clc;
clear;


%��ȡͼ��
Image_Use = imread('xiaowan.bmp');
Pixle = Image_Use;
[Use_HIGH,Use_WIDTH] = size(Image_Use);%����ͷ�ߴ�
subplot(2,2,1);
imshow(Image_Use);
title('ԭʼͼ��');


 %/*!
 %* @brief ͼ���ֵ������
 %*
 %*/
tv=0;
for i = 1:Use_HIGH
    for j = 1:Use_WIDTH
         tv = tv + double(Image_Use(i,j));
    end
end

GaveValue = tv/Use_HIGH/Use_WIDTH;     %/* ƽ���Ҷ� */
Threshold = GaveValue*7/10 + 10;        %/* ��ֵ�Ҷȱ��� */

%��ֵ������Ƚ�
for i = 1:Use_HIGH
    for j =1:Use_WIDTH
        if(Image_Use(i,j) >Threshold)
            Pixle(i,j) = 255;
        else
            Pixle(i,j) = 0;
        end
    end
end

%/*!
%* @brief �������Ϸ���Χ��������
%*/
for i = 2:Use_HIGH-1
    for j = 2:Use_WIDTH-1
        if (Pixle(i,j)==0)&&(double(Pixle(i-1,j))+double(Pixle(i+1,j))+double(Pixle(i,j+1))+double(Pixle(i,j-1))>255*2)
            Pixle(i,j) = 255;
        end
        if (Pixle(i,j)==255)&&(double(Pixle(i-1,j))+double(Pixle(i+1,j))+double(Pixle(i,j+1))+double(Pixle(i,j-1))<255*2)
             Pixle(i,j) = 0;
        end
    end
end

subplot(2,2,2);
imshow(Pixle);     
title('��ֵ��+������');  

mid = 46;

left(Use_HIGH) = 1;
right(Use_HIGH) = Use_WIDTH;
midline(Use_HIGH) = mid;

%����Ѱ��
for i = Use_HIGH:-1:1
	left(i) = 1;
    for j = mid:-1:1       %�������
            if Pixle(i,j) == 0
                left(i) = j;
                break
            end
    end

    right(i) = Use_WIDTH;
	for j = mid:Use_WIDTH       %���ұ���
         if Pixle(i,j)==0
            right(i) = j;
            break
         end
    end  

    mid = uint8((left(i)+right(i))/2);
    midline(i) = mid;
    
    Pixle(i,midline(i)) = 0; %��������

end










%����ƫ��ο���
i = 15;
for j = 1:Use_WIDTH
    if Pixle(i,j)==0
        Pixle(i,j) = 255;
    else
        Pixle(i,j) = 0;
    end
end
i = 25;
for j = 1:Use_WIDTH
    if Pixle(i,j)==0
        Pixle(i,j) = 255;
    else
        Pixle(i,j) = 0;
    end
end
i = 35;
for j = 1:Use_WIDTH
    if Pixle(i,j)==0
        Pixle(i,j) = 255;
    else
        Pixle(i,j) = 0;
    end
end
        
    

    
subplot(2,2,4);
imshow(Pixle);     
title('����'); 

%�����������
%�������Ѱ��
j=1;
down = 59;
down = 59;
for k = 1:h-1
    i= h-k+1;
    if image1(i,1)==255
        if image1(i-1,1)==0
            up = i;
            leftwith(j) = down - up;
            j = j+1;
        end
    end
    if image1(i,1)==0
        if image1(i-1,1)==255
            down = i;
        end
    end
end

%�ұ�����Ѱ��
j=1;
down = 59;
down = 59;
for k = 1:h-5
    i= h-k+1;
    if image1(i,94)==255
        if image1(i-1,94)==0
            up = i;
            rightwith(j) = down - up;
            j = j+1;
        end
    end
    if image1(i,94)==0
        if image1(i-1,94)==255
            down = i;
        end
    end
end

%����Ե���ݴ���
lwhite = 0;
[~,arrw] = size(leftwith);
for i = 1:arrw
    if (leftwith(i) > 4)
        lwhite=lwhite+1;        %���׿�
    end
end
%�Ҳ��Ե���ݴ���
rwhite = 0;
[~,arrw] = size(rightwith);
for i = 1:arrw
    if (rightwith(i) > 4)
        rwhite=rwhite+1;
    end
end





if lwhite ==2 && rwhite ==2
    fprintf('ʮ��·��\n');
end

if lwhite ==2 && rwhite ==1
    fprintf('���Բ\n');
end

%clear a image0 j jj k m1 m2 mG P1 P2
%clear pixelCount pixelPro quanzhong Sigma
%clear SigmaMax threshold u0 u1 up w
%clear image1 down h i mid 




%��͸�ӳ���
subplot(2,2,3);
imshow(image3);     
title('��͸��ԭͼ'); 
HH = 180;WW = 60;
image4(HH,WW) = uint8(0);



            
                



            
            


        