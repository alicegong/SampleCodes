%% Online system - Final Version as of 2015 Sept 04
% Folders to be added to path:
% Online System
% DataSet5
%% Camera & Communicataion Setup
close all;
clc;

vid = videoinput('kinect',1,'RGB_640x480');
% vid = videoinput('winvideo', 1);
% set(vid, 'ReturnedColorSpace', 'RGB');
pause(.3);
preview(vid);
pause(3);
figure;

comm = tcpip('172.23.100.229',4242);
flagcomm = 1;
while flagcomm == 1;
    try
        fopen(comm);
        flagcomm = 0;
        disp('Successful Communication')
    catch 
        disp('Unsuccessful Communication')
    end
end
FESstim = 0;

%% Acquire Background

BGrgb = getBG(vid,5); % acquires the image - can be changed to load a background
se90 = strel('line', 4, 90);
se0 = strel('line', 4, 0);
BGdil = imdilate(BGrgb,[se90 se0]);
BGedge = edge(BGrgb(:,:,1))+edge(BGrgb(:,:,2))+edge(BGrgb(:,:,3));
BGdil = imdilate(BGedge,[se90 se0]);

%% Acquire Object Image & Classify

load('maskFeatures.mat');
load('sparseFeatures.mat');
trainingcounter = 1;
begin = 1;
% TimeOut = 5; %secs 

while 1
% IMAGE ACQUISITION
    boolean = 1; %flag
    quit = 0;
    
    while boolean == 1;
    % get user's input of when to start
    prompt = 'Press 1 and OK to start object image collection or Enter < q > to quit';
    inputkey = cell2mat(inputdlg_withTimeout(prompt));
%     fwrite(comm,int8(FESstim));
        if inputkey == '1';
            boolean = 0;
        elseif inputkey == 'q'
            boolean = 0;
            quit = 1;
        elseif begin == 0;
            if isempty(inputkey);
                if exist('FESstim','var');
                    try
                        fwrite(comm,int8(FESstim+1));
                        disp('Timedout - resent FESstim');
                    catch
                        flagcomm = 1;
                        disp('Lost Connection')
                        while flagcomm == 1;
                            try
                                fopen(comm);
                                flagcomm = 0;
                                disp('Reconnected')
                            catch 
                                disp('Attempting Connection...')
                            end
                        end
                        fwrite(comm,int8(FESstim+1));
                        disp('FESstim resent');
                    end
                end
            end
        else
            disp('Invalid Input');
            clear inputkey;
        end
    end
    
    if quit
        fclose(comm);
        break;
    end
    
    begin = 0;
    disp('Now getting obj photos');
    boolean = 1; %flag

    while boolean == 1;
        for n = 1:5
            fiveImages(:,:,:,n) = getframe(vid);
        end
        OBJrgb = avgpic(fiveImages);
        boolean = 0;
    end
    
% SEGMENTATION
    try
        OBJrgb = avgpic(fiveImages);
        sub_bg_obj = imsubtract(OBJrgb, BGrgb);
        sub_obj_bg = imsubtract(BGrgb, OBJrgb);
        subAB = sub_bg_obj+sub_obj_bg;
        subABtophat = imtophat(subAB,strel('disk',12));

        OBJedge = zeros(480,640);

        for n = 1:3
            [~, thresh] = edge(subAB(:,:,n));
            OBJedge = OBJedge + edge(subAB(:,:,n),thresh*0.9);
        end
        % ^ edge detection on each layer - R,G,B

        grayAB = rgb2gray(subAB);
        OBJedge = OBJedge + edge(grayAB);
        % edge detection on grayscale image

        OBJedge = OBJedge>=2;
        % ^ only edges dectected more than twice are kept 
        % (shadows are unlikely to be detected more than twice)
        OBJedge = OBJedge - BGdil;
        % ^ subtract dilated edge detection of the background

        OBJfill = dilFillHoles(OBJedge,4,4);
        OBJfill = dilFillHoles(OBJedge,4,4);
        bwOBJ = getblobs(OBJfill,6);

        OBJfill = dilFillHoles(bwOBJ,8,8);

        bwOBJ = imclose(bwOBJ,strel('disk',10));
        bwOBJ = imfill(bwOBJ, 'holes');

        OBJ1 = getblobs(bwOBJ,2) - getblobs(bwOBJ,1);
        OBJ2 = getblobs(bwOBJ,1);
        [max_val maxX] = max(max(grayAB));
        [max_val maxY] = max(grayAB(:,maxX));
        % choose the blob based on the brightest point after image subtraction
        chooseBlob = zeros(480,640);
        chooseBlob(maxY, maxX) = 1;
        bwOBJ = max(max(OBJ1.*chooseBlob))*OBJ1 + max(max(OBJ2.*chooseBlob))*OBJ2;

        % final wrap up
        rgbORI = OBJrgb;
        bwOBJmask(:,:,1) = bwOBJ; bwOBJmask(:,:,2) = bwOBJ; bwOBJmask(:,:,3) = bwOBJ;
        rgbOBJ = OBJrgb.*uint8(bwOBJmask);
        
        imshow(bwOBJ);
    % FEAUTURE EXTRACTION & CLASSIFICATION
    % For more customization/older scripts, look in the folder "CH & draft
    % scripts".

        disp('classifying...');

        F = get_feature_vector(bwOBJ,rgbOBJ);
        [Cp, Cscore] = predict(classifier,F);
        Cps = Cp;

        [score1, col1] = max(Cscore);

        % SPARSE FEATURE MATCHING

        CsToCp = [1,2,3,4,4,5,6,7];
        % ^ class by sparse to class predicted (by machine learning on global)
        % Because nuts and dies are supposed to always use the same grasp and
        % catagorized as the same object, there is a repeat of object 4 in this
        % "conversion from class by sparse features to class by global
        % features" since it is unlikely that they have the same sparse
        % features.
        Nbookmark = [1, 42, 82, 121, 150, 189, 232, 280, 319];
        % ^ name bookmark
        % These are the bookmarks for the first frames of each object in the
        % grand list.
        nonzeroCols = find([Cscore(1:4) Cscore(4:7)]);
        % ^ Find the class with nonzero scores
        % The 4th class is duplicated to accommadate the 8 classes sparse
        % feature classification works with.

        if (size(nonzeroCols,2) > 1) && (score1 < 0.6)
            Objs = {'book'; 'cellphone'; 'cup, handle visible'; 'die';...
                'nut'; 'pencil'; 'soda can'; 'tennis';};

            corners = detectHarrisFeatures(rgb2gray(rgbORI));
            corners = maskValidPoints(corners, bwOBJ);
            [Fex, corners] = extractFeatures(rgb2gray(rgbORI),corners);

            matchedScores = [0 0 0 0 0 0 0 0];

            for c = 1:size(nonzeroCols);
                for m = Nbookmark(nonzeroCols(c)):Nbookmark(nonzeroCols(c)+1)-1
                    pairs = matchFeatures(Fs{m}, Fex);
                    for bin = 1:8 
                        if strcmp(Ns{m},Objs{bin})
                            matchedScores(bin) = matchedScores(bin) + size(pairs,1)/Fs{m}.NumFeatures;
                        end
                    end
                    % ^ This loop accumulates the matched scores of sparse features for each object/class.
                end
            end

            if size(find(matchedScores),2) ~= 0 % if there is nonzero elements found
                [Vs, Cs] = max(matchedScores);
                Cps = CsToCp(Cs);
            end
            % ^ Sometimes the matched scores are zeros for all classes
            % because the object has very few sparse feature interest
            % points (eg, nut, tennis, pencil). In that case, without this
            % if condition, max(matchedScores) will be 1, which is
            % incorrect. (For an array like [0 0 0], MATLAB outputs the
            % location of the maxima to be 1...)
        end

    %     Objs = {'book'; 'cellphone'; 'cup, handle visible'; 'die';...
    %             'nut'; 'pencil'; 'soda can'; 'tennis';};
        ObjToGrib = [3,3,1,0,1,2,2]; % die and nut are both 1
        if iscell(Cps)
            Cps = str2num(cell2mat(Cps));
        end
        FESstim = ObjToGrib(Cps);
        trainingBW(:,:,trainingcounter) = bwOBJ;
        trainingcounter = trainingcounter+1;
    catch
        ('error in segmentation please move object')
    end
    fwrite(comm,int8(FESstim+1));
    FESstim+1
end



