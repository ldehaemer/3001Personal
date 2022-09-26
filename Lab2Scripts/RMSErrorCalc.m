
load('Data/OveralHTMatricies.mat')
HTMatriciesActual = zeros(4,4,10);
HTMatriciesGoal = zeros(4,4,10);
for i = 0:9
    HTMatriciesGoal(:,:,i+1) = HTMatrixActualGoal((i*8+1):(i*8+4),:); %Extract All the Goal HT matricies
    HTMatriciesActual(:,:,i+1) = HTMatrixActualGoal((i*8+5):(i*8+8),:);%Extract All the actual HT matricies
end
VectorTipPositionsActual = zeros(3,10); %intiialize the end vector actual
VectorTipPositionsGoal = zeros(3,10); %initialize the end vector goal 
for i = 1:10
    %Extract the translation vector from the HT matricies for the actual and goal HT matrix
    VectorTipPositionsGoal(:,i) = HTMatriciesGoal(1:3,4,i); 
    VectorTipPositionsActual(:,i) = HTMatriciesActual(1:3,4,i);
end
%Calculate the difference vector and initialize an array of these vectors'
%magnitudes
VectorDifference = VectorTipPositionsActual - VectorTipPositionsGoal;
VectorMagnitudes = zeros(1,10);
for i = 1:10
    VectorMagnitudes(1,i) = sqrt(VectorDifference(1,i)^2 + VectorDifference(2,i)^2 + VectorDifference(3,i)^2);
end

VectorMagnitudes = VectorMagnitudes.*VectorMagnitudes; %element-wise multiplication
VectorMagnitudes = VectorMagnitudes/10;
RMSError = sqrt(sum(VectorMagnitudes)) %calc RMS error

clear