function columnIndex = getColumnIndex(columnName, cellArrayOfColumnNames)

columnIndex = 0;

for i=1:1:length(cellArrayOfColumnNames)
   if( strcmp(columnName, cellArrayOfColumnNames{i}) == 1)
      columnIndex = i; 
   end
end

