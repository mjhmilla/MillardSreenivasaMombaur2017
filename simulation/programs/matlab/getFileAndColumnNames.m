function [data colNames] = getFileAndColumnNames(folder,dataFile,colNamesFile) 



data = csvread([folder,dataFile]);
headerFormat = '';
for i=1:1:( size(data,2) - 1)
    headerFormat = [headerFormat,'%s '];
end
headerFormat = [headerFormat,'%s'];
headerFileId = fopen([folder,colNamesFile]);
colNames = textscan(headerFileId,headerFormat);

assert(size(data,2) == length(colNames),'Augmented File not sized correctly');
assert(isempty(colNames{end}) == 0, 'Augmented File not sized correctly');