clc;
clear;
 
IMG_HIGH = 240; 
IMG_WIDTH = 376; 

Image = imread('�����·.bmp');

 subplot(2,2,1);
 imshow(Image);
 title('ԭʼͼ��');
 
 
 img_roadtype = 0;
 %�����������ز��Դ���
    leftslide1 = 0;
    leftslide2 = 0;
    leftslide3 = 0;
    rightslide1 = 0;
    rightslide2 = 0; 
    rightslide3 = 0;      
%ȡ����λ�õ�������� 
for i = IMG_HIGH:-1:50
    if Image(i,10) == 255 && Image(i-1,10) == 0
        leftslide1 = leftslide1 + 1;    
    end
end
for i = IMG_HIGH:-1:50
    if Image(i,15) == 255 && Image(i-1,15) == 0
        leftslide2 = leftslide2 + 1;
    end
end  
for i = IMG_HIGH:-1:50
    if Image(i,20) == 255 && Image(i-1,20) == 0
        leftslide3 = leftslide3 + 1;
    end
end
    
  for i = IMG_HIGH:-1:50
     if Image(i,IMG_WIDTH-10) == 255 && Image(i-1,IMG_WIDTH-10) == 0
        rightslide1 = rightslide1 + 1;
     end
  end
  for i = IMG_HIGH:-1:50
     if Image(i,IMG_WIDTH-15) == 255 && Image(i-1,IMG_WIDTH-15) == 0
        rightslide2 = rightslide2 + 1;
     end
  end
  for i = IMG_HIGH:-1:50
     if Image(i,IMG_WIDTH-20) == 255 && Image(i-1,IMG_WIDTH-20) == 0
        rightslide3 = rightslide3 + 1;
     end
  end
 
  
 if leftslide1 == 2 && rightslide1 ==1
     img_roadtype = 6;
     disp('��໷��')
 end
 if leftslide1 == 2 && rightslide1 ==2
     img_roadtype = 3;
     disp('ʮ��·��')
 end
 
 %/* ����������Ϣ */
midline(IMG_HIGH)   = int16(0);
leftline(IMG_HIGH)    = int16(0);
rightline(IMG_HIGH) = int16(0);

%����У��ǰ��Ԥ��������򵥵� ����ҳ��Զ�
mid = int16(IMG_WIDTH/2);
for i = IMG_HIGH:-1:1
	leftline(i) = 1;
    for j = mid:-1:1       %�������
            if Image(i,j) == 0
                leftline(i) = j;
                break
            end
    end

    rightline(i) = IMG_WIDTH;
    for j = mid:IMG_WIDTH   %���ұ���
         if Image(i,j) == 0
            rightline(i) = j;
            break
         end  
    end

    mid = uint16((leftline(i)+rightline(i))/2);

    if leftline(i) == rightline(i)
        midline(i) = -1;    %%��Ч�������Ѿ�Ѱ�ҽ���  ���ߺͱ��߽���
        
    else
        midline(i) = mid;       
    end
    
end


%�ȿ����ұ��ߵĽ��������
for i = IMG_HIGH:-1:1
    if midline(i) == -1
        break;
    end
end


%�����߽���λ����Ұ̫���£��϶������
if i>90
    disp('���');
    if midline(i+1) > IMG_WIDTH/2
        disp('����')
    else
        disp('����')
    end

else%�������
    
    if midline(i+1) < IMG_WIDTH/2
        disp('·���󣬳�����');
        p_line = rightline;
    else
        disp('·���ң�������');
        p_line = leftline;
    end
    
    %ѡһ���߶��ж�·������
    k1 = p_line(80) - p_line(120);
    k2 = p_line(120) - p_line(160);  
    deltaK = k1 - k2;
    
   %�����Զ����б��k1,������б��k2,����б�ʵı仯��deltaK
   if deltaK >= -6 && deltaK <= 6
       disp('ֱ��');
   else
       disp('���');
       if k1 < 0
           roadfalg = 2;
           disp('����');
       else
           roadfalg = 2;
           disp('����');
       end
   end 
end

