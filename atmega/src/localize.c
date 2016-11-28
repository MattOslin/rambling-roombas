#include "localize.h"

#define eB 0.1
const uint16_t distMatP[4][4] = {
						{  (1+eB)*0,(1+eB)*100,(1+eB)*45,(1+eB)*55},
						{(1+eB)*100,  (1+eB)*0,(1+eB)*90,(1+eB)*70},
						{ (1+eB)*45, (1+eB)*90, (1+eB)*0,(1+eB)*80},
						{ (1+eB)*55, (1+eB)*70,(1+eB)*80, (1+eB)*0}
					};

const uint16_t distMatM[4][4] = {
						{  (1-eB)*0,(1-eB)*100,(1-eB)*45,(1-eB)*55},
						{(1-eB)*100,  (1-eB)*0,(1-eB)*90,(1-eB)*70},
						{ (1-eB)*45, (1-eB)*90, (1-eB)*0,(1-eB)*80},
						{ (1-eB)*55, (1-eB)*70,(1-eB)*80, (1-eB)*0}
					};

const float angMat[4][4] = {
						{      0, 1.5708, 0.4589,  2.2919},
						{-1.5708,      0,-1.1060, -2.1272},
						{-2.6827, 2.0356,      0,  2.8670},
						{-0.8497, 1.0144,-0.2746,		0}
					};

bool determine_position(pos* posStruct, unsigned int* blobs, uint8_t badIdx, uint8_t* order);
bool determine_order(unsigned int* blobs, uint8_t badIdx, uint8_t* order);
bool check_order(unsigned int* blobs, uint8_t badIdx, uint8_t* order);


void localize_init(void) {
	while(m_wii_open() == 0);
}

bool localize_wii(pos* posStruct) {
	bool localizeSuccessful = false;
	static uint8_t blobOrder[] = {0xFF, 0xFF, 0xFF, 0xFF};
	uint16_t wiiBuffer[12];
	m_wii_read(wiiBuffer);

	int i; // Loop indexing

	// Figure out number of bad blobs, along with their location
	// Also check if bad blobs are in same slots as last time, which will
	// be needed for checking if we still have a guess at the blob order
	uint8_t badBlobN = 0;
	uint8_t badIdx = 4;
	static uint8_t lastBadBlobN = 0;
	static uint8_t lastBadIdx = 4;

	for (i = 0; i < 4; i++) {
		m_usb_tx_string(" ");
		m_usb_tx_int(wiiBuffer[3*i]);
		m_usb_tx_string(" ");
		m_usb_tx_int(wiiBuffer[3*i+1]);
		m_usb_tx_string(" ");
		m_usb_tx_int(wiiBuffer[3*i+2]);

		if (wiiBuffer[3*i+1] == 1023) {
			badBlobN++;
			badIdx = i;
		} 
	}

	static bool knownBlobOrder = false;
	knownBlobOrder = knownBlobOrder &&  (badBlobN < 2) && (badBlobN != lastBadBlobN || badIdx == lastBadIdx);
	if (badBlobN > 1) {
		// Panic
		localizeSuccessful = false;
	} else if (knownBlobOrder && check_order(wiiBuffer, badIdx, blobOrder)) {
		// Blob order is known and correct
		localizeSuccessful = determine_position(posStruct, wiiBuffer, badIdx, blobOrder);

	} else {
		// Blob order unknown or incorrect, redetermine
		knownBlobOrder = determine_order(wiiBuffer, badIdx, blobOrder);
		if(!knownBlobOrder) {
			localizeSuccessful = false;
		} else {
			localizeSuccessful =  determine_position(posStruct, wiiBuffer, badIdx, blobOrder);
		}
	}

	

	return localizeSuccessful;
}

bool determine_position(pos* posStruct, unsigned int* blobs, uint8_t badIdx, uint8_t* order) {

	uint8_t missingPoint = 4;
	uint8_t ptA = 0;
	uint8_t ptB = 1;
	uint8_t i;
	uint8_t pointIdx[4];
	for (i = 0; i < 4; i++) {
		pointIdx[order[i]] = i;
	}

	if (badIdx < 4) {
		missingPoint = order[badIdx];
	}

	if (missingPoint == 0) {
		ptA = 2;
	} else if (missingPoint == 1) {
		ptB = 2;
	}

	int16_t diffX = blobs[3*pointIdx[ptB]] - blobs[3*pointIdx[ptA]];
	int16_t diffY = blobs[3*pointIdx[ptB]+1] - blobs[3*pointIdx[ptA]+1];
	posStruct->th = -(angMat[ptA][ptB] - atan2(diffY, diffX));

	double cosTH = cos(-posStruct->th);
	double sinTH = sin(-posStruct->th);
	double vX, vY, yShift;

	if (missingPoint != 0) {
		vX = blobs[3*pointIdx[0]] - 512;
		vY = blobs[3*pointIdx[0]+1] - 384;
		yShift = 50;
	} else {
		vX = blobs[3*pointIdx[1]] - 512;
		vY = blobs[3*pointIdx[1]+1] - 384;
		yShift = -50;
	}

	posStruct->x = 1024 - ((cosTH*vX - sinTH*vY) + 512);
	posStruct->y = 768 - ((sinTH*vX + cosTH*vY) + 384 + yShift);
	return true;
}

bool determine_order(unsigned int* blobs, uint8_t badIdx, uint8_t* order) {
	uint16_t dists[3];
	int i,j;
	uint8_t distIdx = 0;
	uint8_t skipIdx = badIdx > 3 ? 3 : badIdx;

	uint8_t minIdx = 0;
	uint8_t maxIdx = 0;
	uint16_t maxDist = 0;
	uint16_t minDist = 0xFFFF;

	// Calculate the 3 pairwise distances for the 3 point set (if all points 
	// are good, then just drop the point in the last slot)
	// While doing so, find the max/min distances and their locations
	for (i = 0; i < 3; i++) {
		if (i != skipIdx) {
			for(j = i+1; j < 4; j++) {
				if (j != skipIdx) {
					int16_t diffX = (int16_t) blobs[3*i]-blobs[3*j];
					int16_t diffY = (int16_t) blobs[3*i+1]-blobs[3*j+1];
					uint16_t tempDist = sqrt(((long)diffX)*diffX+((long)diffY)*diffY);
					dists[distIdx] = tempDist;

					if (tempDist > maxDist) {
						maxDist = tempDist;
						maxIdx = distIdx;
					}
					if (tempDist < minDist) {
						minDist = tempDist;
						minIdx = distIdx;
					}

					distIdx++;
				}
			}
		}
	}

	// Calculate the ratio of the two shorter sides to the long side
	// These ratios can uniquely identify the triangles, and should hopefully
	// be more robust to sensor noise
	uint8_t ratio1 = (100*minDist)/maxDist;
	uint8_t ratio2 = (100*dists[3-minIdx-maxIdx])/maxDist;
	int8_t orderShift1, orderShift2;
	if (ratio1 > 70 && ratio2 > 70) {
		// Points (1,2,3)
		orderShift1 = 1;
		orderShift2 = 1;
	} else if (ratio1 < 50 && ratio2 > 80) {
		// Points (0,1,2)
		orderShift1 = 0;
		orderShift2 = 0;
	} else if (ratio1 > 50 && ratio2 < 80) {
		// The below sets of points are (annoyingly) similar triangles.
		// To get around this, also use maxDist to distinguish
		if (maxDist > 85) {
			// Points (0,1,3)
			orderShift1 = 0;
			orderShift2 = 1;
		} else {
			// Points (0,2,3)
			orderShift1 = 2;
			orderShift2 = -2;
		}
	} else {
		// Something screwy happened with the reading, panic
		return false;
	}

	// This is the hardest part to explain. There is a correspondence between
	// the location of the min and max distances and the order of the points in the
	// buffer. It is easiest to see by drawing a picture. The "orderShift" variables
	// are also a result of this corespondence and basically require the picture
	// and pseudo-truth tables to make any sense.
	switch (maxIdx) {
		case 0:
			if (minIdx == 1) {
				order[0] = 0; order[1] = 1; order[2] = 2;
			} else {
				order[0] = 1; order[1] = 0; order[2] = 2;
			}
			order[0] += orderShift1;
			order[1] += orderShift1;
			order[2] += orderShift2;
			break;
		case 1:
			if (minIdx == 0) {
				order[0] = 0; order[1] = 2; order[2] = 1;
			} else {
				order[0] = 1; order[1] = 2; order[2] = 0;
			}
			order[0] += orderShift1;
			order[1] += orderShift2;
			order[2] += orderShift1;
			break;
		case 2:
			if (minIdx == 0) {
				order[0] = 2; order[1] = 0; order[2] = 1;
			} else {
				order[0] = 2; order[1] = 1; order[2] = 0;
			}
			order[0] += orderShift2;
			order[1] += orderShift1;
			order[2] += orderShift1;
			break;
	}

	// The above switch statement ignored where the bad index was, so here
	// we correct for that by shifting things around. Also sum the values 
	// in order so that we can calculate the single remaining value.
	uint8_t idxSum = 0;
	for (i = 3; i >= 0; i--) {
		if (i == skipIdx) {
			continue;
		} 

		if (i > skipIdx) {
			order[i] = order[i-1];
		} 

		idxSum += order[i];
	}

	if (idxSum >= 3 && idxSum <= 6) {
		order[skipIdx] = 6 - idxSum;
		return true;
	} else {
		return false;
	}
}

bool check_order(unsigned int* blobs, uint8_t badIdx, uint8_t* order) {
	bool orderGood = true;
	uint8_t i[3] = {0,0,0};
	uint8_t j[3] = {1,2,3};
	switch (badIdx) {
		case 0:
			i[0] = 1; i[1] = 1; i[2] = 2;
			j[0] = 2; j[2] = 3; j[3] = 3;
			break;
		case 1:
			i[0] = 2;
			j[0] = 3;
			break;
		case 2:
			i[1] = 1;
			j[1] = 3;
			break;
		case 3:
			i[2] = 1;
			i[3] = 2;
			break;
		case 4:
			// Initialized order is fine in this case
			break;
	}

	uint8_t k;
	for (k = 0; k < 3; k++) {
		uint8_t blobI = order[i[k]];
		uint8_t blobJ = order[j[k]];
		int16_t diffX = (int16_t) blobs[3*i[k]]-blobs[3*j[k]];
		int16_t diffY = (int16_t) blobs[3*i[k]+1]-blobs[3*j[k]+1];
		uint16_t dist = sqrt(((long)diffX)*diffX+((long)diffY)*diffY);
		orderGood = orderGood && (dist >= distMatM[blobI][blobJ]) && (dist <= distMatP[blobI][blobJ]);
	}

	return orderGood;
}