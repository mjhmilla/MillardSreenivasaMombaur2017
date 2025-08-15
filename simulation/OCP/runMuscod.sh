#!/bin/bash
clear

#Problem specific user-settable variables
problemName="wholebodylifter2d"
rootDir="$(pwd)"
runDir=$rootDir/build
backupDir="$rootDir"/intermediateResults
desiredKKTTOL=1E-5
maxSQPIterations=220
maxOuterLoopIterations=10

#Internal variables - do not touch
KKTTOL=1e100
iter=1

mkdir $backupDir

echo $problemName

#Make a first attempt. If it fails to reach the desired 
#kktt in maxSQPIterations iterations try the problem again with a cold start.
#Why a cold start? After many iterations you might observe the kkt tolerance
#getting worse when the problem is close to convergence. This can happen
#because Muscod's numerical estimate of the Hessian has been corrupted.
cd $runDir
  muscod_release -a"$desiredKKTTOL" -i"$maxSQPIterations" "$problemName"
  cd RES
	  kkttlineNo="$(grep -w -n 'kktt' "$problemName".txt | cut -f1 -d:)"
	  ((kkttlineNo++))
	  KKTTOL="$(sed -n "$kkttlineNo"p "$problemName".txt)"
cd $backupDir
mkdir "RES_$iter"
cp -r "$runDir/RES"/*  "$backupDir/RES_$iter"/
((iter++))

echo $KKTTOL
echo $desiredKKTTOL
isTolMet=$(awk -vn1="$KKTTOL" -vn2="$desiredKKTTOL" 'BEGIN{print (n1<=n2)?1:0 }')
echo $isTolMet

while [ "$isTolMet" -eq 0 ] && [ $iter -le $maxOuterLoopIterations ]; do
	cd $runDir	
	  muscod_release -a"$desiredKKTTOL" -c -i"$maxSQPIterations" "$problemName"
	  cd RES
		  kkttlineNo="$(grep -w -n 'kktt' "$problemName".txt | cut -f1 -d:)"
		  ((kkttlineNo++))
		  KKTTOL="$(sed -n "$kkttlineNo"p "$problemName".txt)"
		  isTolMet=$(awk -vn1="$KKTTOL" -vn2="$desiredKKTTOL" 'BEGIN{print (n1<=n2)?1:0 }')
	cd $backupDir
	mkdir "RES_$iter"
	cp -r "$runDir/RES"/*  "$backupDir/RES_$iter"/
	((iter++))
done

echo $iter
cd ..
