 function varargout = interfaz(varargin)
% INTERFAZ MATLAB code for interfaz.fig
%      INTERFAZ, by itself, creates a new INTERFAZ or raises the existing
%      singleton*.
%
%      H = INTERFAZ returns the handle to a new INTERFAZ or the handle to
%      the existing singleton*.
%
%      INTERFAZ('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFAZ.M with the given input arguments.
%
%      INTERFAZ('Property','Value',...) creates a new INTERFAZ or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before interfaz_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to interfaz_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help interfaz

% Last Modified by GUIDE v2.5 27-Mar-2016 08:52:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interfaz_OpeningFcn, ...
                   'gui_OutputFcn',  @interfaz_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before interfaz is made visible.
function interfaz_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interfaz (see VARARGIN)

% Choose default command line output for interfaz
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interfaz wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = interfaz_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
set(handles.text15,'String','SELECCIONE LA IMAGEN A ANALIZAR')
set(handles.text15,'ForegroundColor',[1 0 0]);
z=clock;
z1=num2str(z(1));
z2=num2str(z(2));
z3=num2str(z(3));
z4=num2str(z(4));
z5=num2str(z(5));

cloc1=strcat('FECHA :',z1,'  /  ',z2,'  /  ');
cloc2=strcat(z3,'   HORA:   ',z4,'  :  ');
cloct=strcat(cloc1,cloc2,z5);
set(handles.text17,'String',cloct);


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global fullpathname;
[filename pathname] = uigetfile({'.jpg'},'File Selector');
fullpathname = strcat(pathname, filename);
set(handles.edit1,'String',fullpathname)
set(handles.edit2,'String','0')
set(handles.edit3,'String','0')
set(handles.edit4,'String','0')
set(handles.edit5,'String','0')
set(handles.edit12,'String','0')
set(handles.text15,'String','Imagen Seleccionada')
set(handles.text15,'ForegroundColor',[1 0 0]);



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.text15,'String','EJECUCIÓN EN PROCESO')
set(handles.text15,'ForegroundColor',[1 0 0]);

checkval=get(handles.checkbox2,'value');


global fullpathname;
clc
close(figure (1))
close(figure (2))
close(figure (3))
close(figure (4))
close(figure (5))
milimetros=get(handles.edit6,'String')
milimetros=str2double(milimetros)
imagen = imread(fullpathname);
figure()
recorte = imcrop(imagen);

if checkval==0
    
    set(handles.edit12,'String','N/A');
    set(handles.edit6,'String','N/A');

end
recorte2 =imcrop(imagen);
w=rgb2gray(recorte2);
BW = w >100;
%BW =not(BW);
stats = regionprops(BW,'BoundingBox');
recuadro=stats.BoundingBox;



%BW=not(BW);
%stats = regionprops(BW,'BoundingBox');
%recuadro=stats.BoundingBox;
%figure(2)
%imshow(BW)

ww=rgb2gray(recorte);
BWW = ww >100;


%segmentacion
imshow(imagen)
close(figure (1))
close(figure (2))
%Im=rgb2gray(imagen);
Im=rgb2gray(recorte);
inversa=not(BWW);
stats = regionprops(inversa,'Centroid');
centre=stats.Centroid;
tam=size(recorte);
mask1=zeros(tam(1),tam(2));
centre1=round(centre(1,1));
centre2=round(centre(1,2));
%mask1(centre(1),centre(2))=1;
fi=ones(5);
p1=centre1+4;
p2=centre2+4;
%mask1(1:20,1:20)= fi;
mask1(centre1:p1,centre2:p2)= fi;
%mask = roipoly;
close(figure(1))

maxIterations = 1500; 
bw = activecontour(Im, mask1, maxIterations, 'Chan-Vese', 'ContractionBias',0.005);
  se = strel('disk',2);
 % bw= imerode(bw,se);
bw = imdilate(bw,se);
bw=not(bw);
% Display segmented image
segmentacion=figure(); 
imshow(recorte,[]);hold on;contour(bw,'r','LineWidth',1);
str = 'RESULTADO SEGMENTACION';
title(str,'Color','b','FontSize',9);
saveas(segmentacion,'Segmentacion.png')


%ww=rgb2gray(recorte);
%BWW = ww >100;
imagen_r = recorte(:,:,1);
imagen_g = recorte(:,:,2);
imagen_b = recorte(:,:,3);
mask = strel('disk',8);
im2 = imerode(bw,mask); %BWW erosion continua
im3 = imdilate(im2,mask);

im22 = imerode(BW,mask);
im33 = imdilate(im22,mask);
im33=not(im33);
im3=not(im3);
[LL Ne]=bwlabel(im33);%%1
tamano=regionprops(LL);
%stats=regionprops(im3,'Area');
%areass=stats.Area;
%hold on
%for n=1:size(tamano,1)
%rectangle('position',tamano(n).BoundingBox,'EdgeColor','g','LineWidth',2);
%end
%stats = regionprops(im33,'BoundingBox');
%recuadro=stats.BoundingBox;
im3 = im2uint8(im3);
% %simetria total
% mask = strel('disk',8);
% im2 = imerode(BWW,mask); %erosion continua
% im3 = imdilate(im2,mask); 
% im3=not(im3);
imv = bitand (im3,imagen_g);
imr = bitand (im3,imagen_r);
ima = bitand (im3,imagen_b);
dimen = size(im3);
dim=dimen(1)*dimen(2);
total_a = 0;
total_a = im2double(total_a);
for i=1: dim(1)
    total_a = ima(i)+total_a;
    if (ima(i)~=0)
    end
end
sa=sum(ima);
sa1=sum(sa);
im2 = imerode(bw,mask); %erosion continua
figure()
imshow(bw)
figure()
imshow(im2)
im3 = imdilate(im2,mask); 
stats=regionprops(im3,'Area');
area = stats.Area;
recv=recuadro(1,4);
recuadro
recv
%milimetros=milimetros/1000;
fff=milimetros/recv;
fff=fff*fff;
fff
area
tamao_real=(area(1))*fff;%%tamaño Real
tamao_real


promedio_azul=sa1/area(1);
promedio_azul=(promedio_azul*100)/255;
promedio_azul
sv=sum(imv);
sv1=sum(sv);
promedio_verde=sv1/area(1);
promedio_verde=(promedio_verde*100)/255;
promedio_verde
sr=sum(imr);
sr1=sum(sr);
promedio_rojo=sr1/area(1);
promedio_rojo=(promedio_rojo*100)/255;
promedio_rojo
totala(:,:,1)=imr;
totala(:,:,2)=imv;
totala(:,:,3)=ima;
%figure(4)
%imshow(totala),title('total vuelta')
%homogeneidad color & realce
med=size(totala);
tam1=0;
tam2=0;
tam3=0;
%figure()
%imshow(totala)
for a=1:1:med(1),
    for b=1:1:med(2),
           
        if (imr(a,b)~=0)
        tam1=tam1+1;
        end 
        if (imv(a,b)~=0)
        tam2=tam2+1;
        end
        if (ima(a,b)~=0)
        tam3=tam3+1;
        end
        
        
    end
      
end

vec1=zeros(1,tam1);
vec2=zeros(1,tam2);
vec3=zeros(1,tam3);
pos1=1;
pos2=1;
pos3=1;
for a=1:1:med(1),
    
    for b=1:1:med(2),
     
        if (imr(a,b)~=0)
        vec1(pos1)=imr(a,b);
        pos1=pos1+1;
       
        end
        if (imv(a,b)~=0)
        vec2(pos2)=imv(a,b);
        pos2=pos2+1;      
        end 
        if (ima(a,b)~=0)
        vec3(pos3)=ima(a,b);
        pos3=pos3+1;
        end      
    end
end
des1=std(vec1);
mean1=mean(vec1);
des2=std(vec2);
mean2=mean(vec2);
des3=std(vec3);
mean3=mean(vec3);

tamt=1;

for a=1:1:med(1),
    
    for b=1:1:med(2),
        
     for c=1:1:med(3),   
        if (totala(a,b,c)~=0)
        tamt=tamt+1;
        end 
     end        
    end
end
n=1;
vect=zeros(1,tamt);
for a=1:1:med(1),
    
    for b=1:1:med(2),
     
     for c=1:1:med(3),
         
        if (totala(a,b,c)~=0)
        vect(n)=totala(a,b,c);
        n=n+1;
       
        end
       
     end  
        
    end
      
end

dest=std(vect);
meant=mean(vect);

for a=1:1:med(1),
    
    for b=1:1:med(2),
        
      for c=1:1:med(3),  
       
      if(totala(a,b,c)>=dest+meant)
        
        totala(a,b,1)=totala(a,b,1)+20;
        totala(a,b,2)=totala(a,b,2)+20;
        totala(a,b,3)=totala(a,b,3)+20;
          
      end
      
      
      end  
        
    end
      
end

totala1=figure();
subplot(1,2,1),imshow(recorte),title('IMAGEN RECORTE')
subplot(1,2,2),imshow(totala),title('IMAGEN REALCE')


saveas(totala1,'Realce.png')
hom1=100-((des1*100)/mean1);
hom2=100-((des2*100)/mean2);
hom3=100-((des3*100)/mean3);
homt=(hom1+hom2+hom3)/3;
des1
des2
des3

set(handles.edit2,'String',homt)
 %%simetria total
 mask = strel('disk',8);
 im2 = imerode(BWW,mask); %erosion continua
 im3 = imdilate(im2,mask); 
 im3=not(im3);
[L Ne]=bwlabel(im3);%%1
propied=regionprops(L);
hold on
for n=1:size(propied,1)
%rectangle('position',propied(n).BoundingBox,'EdgeColor','g','LineWidth',2);
end
stats = regionprops(im3,'Area','BoundingBox');
mascara = stats.BoundingBox;
gx1=mascara(1,1);
gy1=mascara(1,2);
gx2=gx1+mascara(1,3);
gy2=gy1+mascara(1,4);
gx1=round(gx1);
gx2=round(gx2);
gy1=round(gy1);
gy2=round(gy2);
fi=im3([gy1:gy2],[gx1:gx2]);%extraccion figura total
centr =propied.Centroid;
c1x2=centr(1,1);
c1y2=centr(1,2);
c1x2=round(c1x2);
c1y2=round(c1y2);
cuadrante1=im3([gy1:c1y2],[gx1:c1x2]);%extraccion cuadrante 1
c2y1=gy1+(mascara(1,4))/2;
c2y1=round(c2y1);
c2x2=gx1+(mascara(1,3))/2;
c2x2=round(c2x2);
cuadrante2= im3([c2y1:gy2],[gx1:c2x2]);%extraccion cuadrante 2
cuadrante3= im3([gy1:c1y2],[c1x2:gx2]);%extraccion cuadrante 3
cuadrante4= im3([c1y2:gy2],[c1x2:gx2]);%extraccion cuadrante 4
%plot(gx2,c1y2,'*')
%plot(c1x2,gy1,'*')
%tform = maketform('affine',[0 1 0; -1 0 0; 0 0 1]);
tform = maketform('projective',[0 1 0; -1 0 0; 0 0 1]);
cuadrante1=im2uint8(cuadrante1);
cuadrante2=im2uint8(cuadrante2);
cuadrante3=im2uint8(cuadrante3);
cuadrante4=im2uint8(cuadrante4);
im6 = imtransform(cuadrante1,tform);
im7 = imtransform(cuadrante2,tform);
im8 = imtransform(cuadrante3,tform);
im9 = imtransform(cuadrante4,tform);
dimension = size(cuadrante1);
dimension_2 = size(cuadrante2);
dimension_3 = size(cuadrante3);
dimension_4 = size(cuadrante4);
matrizdim_1=[dimension(1),dimension_2(1),dimension_3(1),dimension_4(1)];
matrizdim_2=[dimension(2),dimension_2(2),dimension_3(2),dimension_4(2)];
numeromd1=min(matrizdim_1);
numeromd2=min(matrizdim_2);
cuadrante1 = imcrop(cuadrante1,[0 0 numeromd2 numeromd1]);
cuadrante2 = imcrop(cuadrante2,[0 0 numeromd2 numeromd1]);
cuadrante3 = imcrop(cuadrante3,[0 0 numeromd2 numeromd1]);
cuadrante4 = imcrop(cuadrante4,[0 0 numeromd2 numeromd1]);
dimension = size(cuadrante1);
dimension_2 = size(cuadrante2);
dimension_3 = size(cuadrante3);
dimension_4 = size(cuadrante4);
    for Y = 1: dimension(1),
        for X = 1:dimension(2), 
            cuadrante1_inv(Y,X) = cuadrante1(Y,(dimension(2) - (X-1)));
        end     
    end     
%dimension_2 = size(cuadrante2);
    for Y = 1: dimension_2(1),
        for X = 1:dimension_2(2), 
            cuadrante2_inv(Y,X) = cuadrante2(Y,(dimension_2(2) - (X-1)));
        end     
    end 
    for Y = 1: dimension(1),

        for X = 1:dimension(2), 
            cuadrante1y_inv(Y,X) = cuadrante1((dimension(1) - (Y-1)),X);
        end     
    end
    %dimension_3 = size(cuadrante3);
    for Y = 1: dimension(1),

        for X = 1:dimension_3(2), 
            cuadrante3y_inv(Y,X) = cuadrante3((dimension_3(1) - (Y-1)),X);
        end     
    end
%final1=cuadrante3-cuadrante1_inv;
final1=bitxor(cuadrante1_inv,cuadrante3);
%final2=cuadrante2_inv-cuadrante4;
final2=bitxor(cuadrante2_inv,cuadrante4);
%final3=cuadrante2-cuadrante1y_inv;
final3=bitxor(cuadrante1y_inv,cuadrante2);
%final4=cuadrante3y_inv-cuadrante4;
final4=bitxor(cuadrante3y_inv,cuadrante4);
%stats=regionprops(im3,'Area');
%areass=stats.Area;
areass=area/4;    
areafinal1=regionprops(final1,'area');
%areafinal=sum(sum(im3));
areafinal2=regionprops(final2,'area');
areafinal3=regionprops(final3,'area');
areafinal4=regionprops(final4,'area');
areafinal1=areafinal1(255,1);
areafinal11=struct2cell(areafinal1); 
areafinal11=cell2mat(areafinal11);
simee1=(areafinal11*100)/areass;%  =100
simee1
areafinal2=areafinal2(255,1);
areafinal22=struct2cell(areafinal2); 
areafinal22=cell2mat(areafinal22);
simee2=(areafinal22*100)/areass;%  =100
simee2
areafinal3=areafinal3(255,1);
areafinal33=struct2cell(areafinal3); 
areafinal33=cell2mat(areafinal33);
simee3=(areafinal33*100)/areass;%  =100
simee3
areafinal4=areafinal4(255,1);
areafinal44=struct2cell(areafinal4); 
areafinal44=cell2mat(areafinal44);
simee4=(areafinal44*100)/areass;%  =100
simee4
simeetotal=(simee1+simee2+simee3+simee4)/4;
simeetotal


set(handles.edit3,'String',promedio_rojo)
set(handles.edit4,'String',promedio_verde)
set(handles.edit5,'String',promedio_azul)
set(handles.edit12,'String',tamao_real)
checkval=get(handles.checkbox2,'value');
if checkval==0
    set(handles.edit12,'String','N/A');
    set(handles.edit6,'String','N/A');
end 
set(handles.text15,'String','PROCESO TERMINADO')
set(handles.text15,'ForegroundColor',[0 1 0]);
function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
set(handles.text15,'String','GENERANDO REPORTE')
set(handles.text15,'ForegroundColor',[1 0 0]);
formatos = {'*.txt','txt (*.txt)'}; 
[nomb,ruta] = uiputfile(formatos,'Guardat Reporte'); 
fName = fullfile(ruta,nomb); 
nombre=get(handles.edit10,'String');
edad=get(handles.edit11,'String');
observaciones=get(handles.edit8,'String');
p_h_c=get(handles.edit2,'String');
p_c_r=get(handles.edit3,'String');
p_c_v=get(handles.edit4,'String');
p_c_a=get(handles.edit5,'String');
tamano1f=get(handles.edit12,'String');
fecha=get(handles.text17,'String');
fileID = fopen(fName,'w');
fprintf(fileID,'%s\r\n%s\r\n\r\n','Nombre Del Paciente : ',nombre);
fprintf(fileID,'%s\r\n%s AÑOS\r\n\r\n','Edad Del Paciente : ',edad);
fprintf(fileID,'%s\r\n%s\r\n\r\n','Observaciones : ',observaciones);
fprintf(fileID,'%s\r\n%s PORCIENTO \r\n\r\n','Porcentaje Homegeneidad Del Color Del Melanoma : ',p_h_c);
fprintf(fileID,'%s\r\n%s PORCIENTO \r\n\r\n','Porcentaje De Color Rojo : ',p_c_r);
fprintf(fileID,'%s\r\n%s PORCIENTO \r\n\r\n','Porcentaje De Color Verde : ',p_c_v);
fprintf(fileID,'%s\r\n%s PORCIENTO \r\n\r\n','Porcentaje De Color Azul : ',p_c_a);
fprintf(fileID,'%s\r\n%s milimetros cuadrados\r\n\r\n','Area Del Melanoma : ',tamano1f);
fprintf(fileID,'%s\r\n ',fecha);
fclose(fileID);
type(fName)

seg1=imread('Segmentacion.png');
realce=imread('Realce.png');
imwrite(seg1,strcat(ruta, 'segmentacion',nomb,'.jpg')); 
imwrite(realce,strcat(ruta, 'imagen con realce',nomb,'.jpg'));

set(handles.text15,'String','REPORTE GENERADO')
set(handles.text15,'ForegroundColor',[0 1 0]);
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkbox1

   

function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future       version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
set(handles.edit2,'String','0')
set(handles.edit3,'String','0')
set(handles.edit4,'String','0')
set(handles.edit5,'String','0')
set(handles.edit6,'String','0')
set(handles.edit12,'String','0')
set(handles.edit1,'String','')
set(handles.edit8,'String','')
set(handles.edit10,'String','')
set(handles.edit11,'String','')
set(handles.text15,'String','SELECCIONE LA IMAGEN A ANALIZAR')
close(figure (1))
close(figure (2))
close(figure (3))
close(figure (4))
close(figure (5))
clc



% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)



% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes3


% --- Executes during object creation, after setting all properties.
function axes4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes4
a=imread('i2.png');
image(a)
axis off

