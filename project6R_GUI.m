

function varargout = project6R_GUI(varargin)
% PROJECT6R_GUI MATLAB code for project6R_GUI.fig
%      PROJECT6R_GUI, by itself, creates a new PROJECT6R_GUI or raises the existing
%      singleton*.
%
%      H = PROJECT6R_GUI returns the handle to a new PROJECT6R_GUI or the handle to
%      the existing singleton*.
%
%      PROJECT6R_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJECT6R_GUI.M with the given input arguments.
%
%      PROJECT6R_GUI('Property','Value',...) creates a new PROJECT6R_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before project6R_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to project6R_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help project6R_GUI

% Last Modified by GUIDE v2.5 24-May-2021 13:26:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @project6R_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @project6R_GUI_OutputFcn, ...
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


% --- Executes just before project6R_GUI is made visible.
function project6R_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to project6R_GUI (see VARARGIN)

% Choose default command line output for project6R_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes project6R_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

current_theta = [0 0 0 0 0 0];

handles.current_theta=current_theta;
guidata(hObject,handles)





% --- Outputs from this function are returned to the command line.
function varargout = project6R_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in GoHome.
function GoHome_Callback(hObject, eventdata, handles)
% hObject    handle to GoHome (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

current_theta=handles.current_theta;

theta_pose_home = [0 0 0 0 0 0];

t_final = str2double(handles.t_final.String);


%Get starting position
theta_start = current_theta;

theta_finish = theta_pose_home;

%Start timer to record elapsed time during trajectory motion
tic
dt = toc;

while dt <= t_final
%Get current time since start of time keeping
dt = toc;

%///////////////////////////////////////////////////////////////
%next_traj_thetas = next_theta(theta_start, theta_finish, dt, t_final);
    D = theta_finish - theta_start;
    
    next_traj_thetas = zeros(length(theta_start));
    for i=1:length(theta_start)
        coef_a0 = theta_start(i);
        % recall coef_a1 = 0;
        coef_a2 = (3/(t_final^2))*D(i);
        coef_a3 = -(2/(t_final^3))*D(i);
        next_traj_thetas(i) = coef_a0 + (coef_a2*(dt^2)) + (coef_a3*(dt^3));
    end
%//////////////////////////////////////////////////////////////////////

[TransToJoint_j,end_effector_trans] = next_TransformPose(next_traj_thetas);

%Get coordinate point to put for plotting frame 0
Base_Pos = 0;
    
X = cat(1,Base_Pos,squeeze(TransToJoint_j(1,4,:)));
Y = cat(1,Base_Pos,squeeze(TransToJoint_j(2,4,:)));
Z = cat(1,Base_Pos,squeeze(TransToJoint_j(3,4,:)));
X = cat(1,X,squeeze(end_effector_trans(1,:)'));
Y = cat(1,Y,squeeze(end_effector_trans(2,:)'));
Z = cat(1,Z,squeeze(end_effector_trans(3,:)'));


plot3(X,Y,Z,'r-o','LineWidth',3) 

xlim([-10 10])
ylim([-10 10])
zlim([0 10])

zlabel('z');
ylabel('y');
xlabel('x');
grid on
grid minor
rotate3d on
drawnow

%Get current time since start of time keeping
dt = toc;
end

%{
handles.Px_current.String = num2str(floor(X(7)));
handles.Py_current.String = num2str(floor(Y(7)));
handles.Pz_current.String = num2str(floor(Z(7)));
%}

%update current joint angles
current_theta = theta_finish;
handles.current_theta=current_theta;
guidata(hObject,handles)



function R1C1_Callback(hObject, eventdata, handles)
% hObject    handle to R1C1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R1C1 as text
%        str2double(get(hObject,'String')) returns contents of R1C1 as a double


% --- Executes during object creation, after setting all properties.
function R1C1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R1C1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R1C2_Callback(hObject, eventdata, handles)
% hObject    handle to R1C2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R1C2 as text
%        str2double(get(hObject,'String')) returns contents of R1C2 as a double


% --- Executes during object creation, after setting all properties.
function R1C2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R1C2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R1C3_Callback(hObject, eventdata, handles)
% hObject    handle to R1C3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R1C3 as text
%        str2double(get(hObject,'String')) returns contents of R1C3 as a double


% --- Executes during object creation, after setting all properties.
function R1C3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R1C3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R2C1_Callback(hObject, eventdata, handles)
% hObject    handle to R2C1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R2C1 as text
%        str2double(get(hObject,'String')) returns contents of R2C1 as a double


% --- Executes during object creation, after setting all properties.
function R2C1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R2C1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R2C2_Callback(hObject, eventdata, handles)
% hObject    handle to R2C2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R2C2 as text
%        str2double(get(hObject,'String')) returns contents of R2C2 as a double


% --- Executes during object creation, after setting all properties.
function R2C2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R2C2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R2C3_Callback(hObject, eventdata, handles)
% hObject    handle to R2C3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R2C3 as text
%        str2double(get(hObject,'String')) returns contents of R2C3 as a double


% --- Executes during object creation, after setting all properties.
function R2C3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R2C3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R3C1_Callback(hObject, eventdata, handles)
% hObject    handle to R3C1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R3C1 as text
%        str2double(get(hObject,'String')) returns contents of R3C1 as a double


% --- Executes during object creation, after setting all properties.
function R3C1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R3C1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R3C2_Callback(hObject, eventdata, handles)
% hObject    handle to R3C2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R3C2 as text
%        str2double(get(hObject,'String')) returns contents of R3C2 as a double


% --- Executes during object creation, after setting all properties.
function R3C2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R3C2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R3C3_Callback(hObject, eventdata, handles)
% hObject    handle to R3C3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R3C3 as text
%        str2double(get(hObject,'String')) returns contents of R3C3 as a double


% --- Executes during object creation, after setting all properties.
function R3C3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R3C3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Px_Callback(hObject, eventdata, handles)
% hObject    handle to Px (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Px as text
%        str2double(get(hObject,'String')) returns contents of Px as a double


% --- Executes during object creation, after setting all properties.
function Px_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Px (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Py_Callback(hObject, eventdata, handles)
% hObject    handle to Py (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Py as text
%        str2double(get(hObject,'String')) returns contents of Py as a double


% --- Executes during object creation, after setting all properties.
function Py_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Py (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pz_Callback(hObject, eventdata, handles)
% hObject    handle to Pz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pz as text
%        str2double(get(hObject,'String')) returns contents of Pz as a double


% --- Executes during object creation, after setting all properties.
function Pz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Go_To_Position.
function Go_To_Position_Callback(hObject, eventdata, handles)
% hObject    handle to Go_To_Position (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

P_goal = zeros(3,1);
P_goal(1) = str2double(handles.Px.String);
P_goal(2) = str2double(handles.Py.String);
P_goal(3) = str2double(handles.Pz.String);

R_goal = zeros(3,3);

%{
theta_x = str2double(handles.Rx.String);
theta_y = str2double(handles.Ry.String);
theta_z = str2double(handles.Rz.String);
%}
theta_x = 0;
theta_y = 0;
theta_z = 0;

Rot_x = [1,0,0; 0,cosd(theta_x),-sind(theta_x); 0,sind(theta_x),cosd(theta_x)];
Rot_y = [cosd(theta_y),0,sind(theta_y); 0,1,0; -sind(theta_y),0,cosd(theta_y)];
Rot_z = [cosd(theta_z),-sind(theta_z),0; sind(theta_z),cosd(theta_z),0; 0,0,1];
R_goal = Rot_x*Rot_y*Rot_z;

% display the rotation matrix
%{
handles.R1C1.String = num2str(floor(R_goal(1,1)));
handles.R1C2.String = num2str(floor(R_goal(1,2)));
handles.R1C3.String = num2str(floor(R_goal(1,3)));
handles.R2C1.String = num2str(floor(R_goal(2,1)));
handles.R2C2.String = num2str(floor(R_goal(2,2)));
handles.R2C3.String = num2str(floor(R_goal(2,3)));
handles.R3C1.String = num2str(floor(R_goal(3,1)));
handles.R3C2.String = num2str(floor(R_goal(3,2)));
handles.R3C3.String = num2str(floor(R_goal(3,3)));
%}

t_final = str2double(handles.t_final.String);

T06_goal = zeros(4,4);
T06_goal(1:3,4) = P_goal;
T06_goal(1:3,1:3) = R_goal;
T06_goal(4,4) = 1;

current_theta=handles.current_theta;

%Get starting position
theta_start = current_theta;

%Determine goal joint angles with inverse
%[theta_finish, q_d] = invCalc(T06_goal);
q_angles = inverse_kin(P_goal(1),P_goal(2),P_goal(3));
theta_finish = q_angles*(pi/180);
%Start timer to record elapsed time during trajectory motion
tic
dt = toc;

while dt <= t_final
%Get current time since start of time keeping
dt = toc;

%///////////////////////////////////////////////////////////////
%next_traj_thetas = next_theta(theta_start, theta_finish, dt, t_final);
    D = theta_finish - theta_start;
    
    next_traj_thetas = zeros(length(theta_start));
    for i=1:length(theta_start)
        coef_a0 = theta_start(i);
        % recall coef_a1 = 0;
        coef_a2 = (3/(t_final^2))*D(i);
        coef_a3 = -(2/(t_final^3))*D(i);
        next_traj_thetas(i) = coef_a0 + (coef_a2*(dt^2)) + (coef_a3*(dt^3));
    end
%//////////////////////////////////////////////////////////////////////

[TransToJoint_j,end_effector_trans] = next_TransformPose(next_traj_thetas);

%Get coordinate point to put for plotting frame 0
Base_Pos = 0;
    
X = cat(1,Base_Pos,squeeze(TransToJoint_j(1,4,:)));
Y = cat(1,Base_Pos,squeeze(TransToJoint_j(2,4,:)));
Z = cat(1,Base_Pos,squeeze(TransToJoint_j(3,4,:)));
X = cat(1,X,squeeze(end_effector_trans(1,:)'));
Y = cat(1,Y,squeeze(end_effector_trans(2,:)'));
Z = cat(1,Z,squeeze(end_effector_trans(3,:)'));


plot3(X,Y,Z,'r-o','LineWidth',3) 

xlim([-10 10])
ylim([-10 10])
zlim([0 10])

zlabel('z');
ylabel('y');
xlabel('x');
grid on
grid minor
rotate3d on
drawnow

%Get current time since start of time keeping
dt = toc;
end

%{
handles.Px_current.String = num2str(floor(X(7)));
handles.Py_current.String = num2str(floor(Y(7)));
handles.Pz_current.String = num2str(floor(Z(7)));
%}

%update current joint angles
current_theta = theta_finish;
handles.current_theta=current_theta;
guidata(hObject,handles)










function t_final_Callback(hObject, eventdata, handles)
% hObject    handle to t_final (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of t_final as text
%        str2double(get(hObject,'String')) returns contents of t_final as a double


% --- Executes during object creation, after setting all properties.
function t_final_CreateFcn(hObject, eventdata, handles)
% hObject    handle to t_final (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Rx_Callback(hObject, eventdata, handles)
% hObject    handle to Rx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Rx as text
%        str2double(get(hObject,'String')) returns contents of Rx as a double


% --- Executes during object creation, after setting all properties.
function Rx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Rx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ry_Callback(hObject, eventdata, handles)
% hObject    handle to Ry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ry as text
%        str2double(get(hObject,'String')) returns contents of Ry as a double


% --- Executes during object creation, after setting all properties.
function Ry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Rz_Callback(hObject, eventdata, handles)
% hObject    handle to Rz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Rz as text
%        str2double(get(hObject,'String')) returns contents of Rz as a double


% --- Executes during object creation, after setting all properties.
function Rz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Rz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Px_current_Callback(hObject, eventdata, handles)
% hObject    handle to Px_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Px_current as text
%        str2double(get(hObject,'String')) returns contents of Px_current as a double


% --- Executes during object creation, after setting all properties.
function Px_current_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Px_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Py_current_Callback(hObject, eventdata, handles)
% hObject    handle to Py_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Py_current as text
%        str2double(get(hObject,'String')) returns contents of Py_current as a double


% --- Executes during object creation, after setting all properties.
function Py_current_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Py_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pz_current_Callback(hObject, eventdata, handles)
% hObject    handle to Pz_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pz_current as text
%        str2double(get(hObject,'String')) returns contents of Pz_current as a double


% --- Executes during object creation, after setting all properties.
function Pz_current_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pz_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Rx_current_Callback(hObject, eventdata, handles)
% hObject    handle to Rx_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Rx_current as text
%        str2double(get(hObject,'String')) returns contents of Rx_current as a double


% --- Executes during object creation, after setting all properties.
function Rx_current_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Rx_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ry_current_Callback(hObject, eventdata, handles)
% hObject    handle to Ry_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ry_current as text
%        str2double(get(hObject,'String')) returns contents of Ry_current as a double


% --- Executes during object creation, after setting all properties.
function Ry_current_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ry_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Rz_current_Callback(hObject, eventdata, handles)
% hObject    handle to Rz_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Rz_current as text
%        str2double(get(hObject,'String')) returns contents of Rz_current as a double


% --- Executes during object creation, after setting all properties.
function Rz_current_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Rz_current (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
