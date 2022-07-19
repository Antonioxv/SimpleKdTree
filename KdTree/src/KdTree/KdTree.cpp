 #include "kdtreepch.h"

 #include "KdTree.h"

using namespace Kdtree;


/* Ctor */
KdTree::KdTree(const vector<Point>& vecPts) : KdTree(vecPts, 0, vecPts.size()) {}

KdTree::KdTree(const vector<Point>& vecPts, int left, int right) {  // vecPts: [left, right).
	this->p_vecPts = &vecPts;

	vector<int> indices(vecPts.size());
	for (int i = 0; i < indices.size(); ++i) { indices[i] = i; }
	this->m_indices = indices;

	double min_x = vecPts[left].x, min_y = vecPts[left].y, min_z = vecPts[left].z;
	double max_x = min_x, max_y = min_y, max_z = min_z;
	for (int i = left; i < vecPts.size() && i < right; ++i) {
		min_x = fmin(min_x, vecPts[i].x);
		max_x = fmax(max_x, vecPts[i].x);
		min_y = fmin(min_y, vecPts[i].y);
		max_y = fmax(max_y, vecPts[i].y);
		min_z = fmin(min_z, vecPts[i].z);
		max_z = fmax(max_z, vecPts[i].z);
	}
	AABB* aabb = new AABB(min_x, max_x, min_y, max_y, min_z, max_z);

	this->p_root = BuildKdTree(aabb, left, right, 0);
}


/* Build KdTree */
double KdTree::FindMedian(KdNode* root, int idxMid) {  // Root of a subtree(or the whole tree).
	struct cmpobj
	{
		cmpobj(SPLIT_AXIS axis, const vector<Point>* vecPts) : m_axis(axis), m_vecPts(vecPts) {}
		bool operator() (int i, int j) const
		{
			if (m_axis == SPLIT_AXIS::AXIS_X) { return (*m_vecPts)[i].x < (*m_vecPts)[j].x; }
			if (m_axis == SPLIT_AXIS::AXIS_Y) { return (*m_vecPts)[i].y < (*m_vecPts)[j].y; }
			if (m_axis == SPLIT_AXIS::AXIS_Z) { return (*m_vecPts)[i].z < (*m_vecPts)[j].z; }
			return true;
		}
		SPLIT_AXIS m_axis;
		const vector<Point>* m_vecPts;
	};

	int len = root->idxRight - root->idxLeft; idxMid = root->idxLeft + ((len - 1) >> 1);  // 10 >>> 5th, 4, 9 >>> 5th, 4
	std::nth_element(KdTree::m_indices.begin() + root->idxLeft,
		KdTree::m_indices.begin() + idxMid,
		KdTree::m_indices.begin() + root->idxRight, cmpobj(root->splitAxis, KdTree::p_vecPts));
	int idxPt = KdTree::m_indices[idxMid];

	double median = -1;
	switch (root->splitAxis) {
	case SPLIT_AXIS::AXIS_X:
		median = (*p_vecPts)[idxPt].x;
		break;
	case SPLIT_AXIS::AXIS_Y:
		median = (*p_vecPts)[idxPt].y;
		break;
	case SPLIT_AXIS::AXIS_Z:
		median = (*p_vecPts)[idxPt].z;
		break;
	}
	return median;
}

void KdTree::Divide(KdNode* root, AABB* aabbLeft, AABB* aabbRight) {
	aabbLeft = root->aabb; aabbRight = root->aabb;
	switch (root->splitAxis) {
	case SPLIT_AXIS::AXIS_X:
		aabbLeft->m_trf.x = root->splitLine;
		aabbRight->m_bln.x = root->splitLine;
		break;
	case SPLIT_AXIS::AXIS_Y:
		aabbLeft->m_trf.y = root->splitLine;
		aabbRight->m_bln.y = root->splitLine;
		break;
	case SPLIT_AXIS::AXIS_Z:
		aabbLeft->m_trf.z = root->splitLine;
		aabbRight->m_bln.z = root->splitLine;
		break;
	}
}

KdNode* KdTree::CreateLeaf(int idx) {
	KdNode* node = new KdNode(idx, idx + 1);
	return node;
}

KdNode* KdTree::CreateSubtree(AABB* aabb, int begin, int end) {
	KdNode* node = new KdNode(begin, end);
	node->aabb = aabb;
	return node;
}

KdNode* KdTree::BuildKdTree(AABB* aabb, int left, int right, int depth) {
	// 0) Base: leaf node. 
	if (right - left == 1) {  // Size == 1, [begin, right), 
		return CreateLeaf(left);
	}
	else if (right - left > 1) {  // Size > 1, right - left > 1.
		// 1) Create subtree node.
		KdNode* root = CreateSubtree(aabb, left, right);
		// 2) Specify split axis, simply: x-y-z-x, but it's not always the most efficient one.
		root->splitAxis = (enum SPLIT_AXIS)(depth % 2);
		// 3) Find median and divide subset.
		int mid = left; root->splitLine = FindMedian(root, mid);
		// 4) Divide AABB.
		AABB* aabbLeft = nullptr; AABB* aabbRight = nullptr; Divide(root, aabbLeft, aabbRight);
		// 5) Recursively build lc and rc.
		root->lc = BuildKdTree(aabbLeft, left, mid, depth + 1);
		root->rc = BuildKdTree(aabbRight, mid + 1, right, depth + 1);
		// 6) Return the root.
		return root;
	}
	else { return nullptr; /* Impossible condition */ }
}


/* Range Query */
bool KdTree::Inside(KdNode* v, AABB* aabb) const {
	Point p = (*p_vecPts)[v->idxLeft];
	Point bln = aabb->m_bln, trf = aabb->m_trf;
	// if (p[0] < bln[0] || p[1] < bln[1] || p[2] < bln[2]) { return false; }
	if (p[0] <= bln[0] || p[1] <= bln[1] || p[2] <= bln[2]) { return false; }
	else if (p[0] > trf[0] || p[1] > trf[1] || p[2] > trf[2]) { return false; }
	else{ return true; }	
}

void KdTree::Report(KdNode* leaf, vector<int>& ans) const {
	ReportSubtree(leaf, ans);
}

void KdTree::ReportSubtree(KdNode* subtree, vector<int>& ans) const {
	for (int idx = subtree->idxLeft; idx < subtree->idxRight; ++idx) {
		ans.push_back(idx);
	}
	// std::cout << "The size of ans is: " << ans.size() << std::endl;
}

void KdTree::RangeSearchFromRoot(AABB* R, vector<int>& ans) const {
	KdTree::RangeSearch(this->p_root, R, ans);
}

void KdTree::RangeSearch(KdNode* root, AABB* R, vector<int>& ans) const {
	if (root->IsLeaf()) {  // outDegree == 0
		// if(Inside(root, R)) {
		if (R->Contain(root->aabb)) {
			Report(root, ans);
		}
	}
	else {  // outDegree == 2
		if (R->Contain(root->lc->aabb)) {
			ReportSubtree(root->lc, ans);
		}
		else if (R->Intersect(root->lc->aabb)) {
			RangeSearch(root->lc, R, ans);
		}
		else { /* do nothing */ }

		if (R->Contain(root->rc->aabb)) {  // A
			ReportSubtree(root->rc, ans);
		}
		else if (R->Intersect(root->rc->aabb)) {
			RangeSearch(root->rc, R, ans);
		}
		else { /* do nothing */ }
	}
}


/* Knn Query */
void KdTree::KnnSearchFromRoot(int idxSearchPoint, int k, vector<int>& ans) const {
	std::priority_queue<std::pair<double, int>> pq;  // Big top heap.
	pq.push({ DOUBLE_INFINITY, -1 });  // For comparing.
	KdTree::KnnSearch(this->p_root, idxSearchPoint, k, pq);
	while (pq.empty()) {
		ans.push_back(pq.top().second);
		pq.pop();
	}
}

void KdTree::KnnSearch(KdNode* root, int idxSearchPoint, int k, std::priority_queue<std::pair<double, int>> pq) const {
	if (root->IsLeaf()) { // Leaf node(has no aabb).
		double distance = (*p_vecPts)[root->idxLeft].GetDistanceToAPoint((*p_vecPts)[idxSearchPoint]);
		pq.push({ distance, root->idxLeft });
		while (pq.size() > k) { pq.pop(); }
	}
	else {  // Subtree node(has a aabb).
		double longestDistance = pq.top().first;
		double distance = root->aabb->GetDistanceToAPoint((*p_vecPts)[idxSearchPoint]);
		if (distance > longestDistance) {
			return;
		}
		else {
			// Point of view, the implementation in the reference is not accerlating the process at all.
			KnnSearch(root->lc, idxSearchPoint, k, pq);
			KnnSearch(root->rc, idxSearchPoint, k, pq);
		}
	}
}


///* Radius Query */
//void Kdtree::KdTree::RadiusSearchFromRoot(Point& p, double radius, vector<int>& ans) const
//{
//}
//
//void Kdtree::KdTree::RadiusSearchFromRoot(int idxSearchPoint, double radius, vector<int>& ans) const
//{
//}
//
//void Kdtree::KdTree::RadiusSearch(KdNode* root, Point& p, double radius, vector<int>& ans) const
//{
//}
//
//void Kdtree::KdTree::RadiusSearch(KdNode* root, int idxSearchPoint, double radius, vector<int>& ans) const
//{
//}





