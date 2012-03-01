#ifndef LEASTSQUARESQUAT_H
#define LEASTSQUARESQUAT_H

#include "opencv/cv.h"

using namespace std;

#define VTK_ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);\
        a[k][l]=h+s*(g-h*tau)

template<class T>
int JacobiN(T **a, int n, T *w, T **v)
{
  int i, j, k, iq, ip, numPos;
  T tresh, theta, tau, t, sm, s, h, g, c, tmp;
  T bspace[4], zspace[4];
  T *b = bspace;
  T *z = zspace;

  // only allocate memory if the matrix is large
  if (n > 4)
    {
    b = new T[n];
    z = new T[n]; 
    }

  // initialize
  for (ip=0; ip<n; ip++) 
    {
    for (iq=0; iq<n; iq++)
      {
      v[ip][iq] = 0.0;
      }
    v[ip][ip] = 1.0;
    }
  for (ip=0; ip<n; ip++) 
    {
    b[ip] = w[ip] = a[ip][ip];
    z[ip] = 0.0;
    }

  // begin rotation sequence
  for (i=0; i<20; i++) 
    {
    sm = 0.0;
    for (ip=0; ip<n-1; ip++) 
      {
      for (iq=ip+1; iq<n; iq++)
        {
        sm += fabs(a[ip][iq]);
        }
      }
    if (sm == 0.0)
      {
      break;
      }

    if (i < 3)                                // first 3 sweeps
      {
      tresh = 0.2*sm/(n*n);
      }
    else
      {
      tresh = 0.0;
      }

    for (ip=0; ip<n-1; ip++) 
      {
      for (iq=ip+1; iq<n; iq++) 
        {
        g = 100.0*fabs(a[ip][iq]);

        // after 4 sweeps
        if (i > 3 && (fabs(w[ip])+g) == fabs(w[ip])
        && (fabs(w[iq])+g) == fabs(w[iq]))
          {
          a[ip][iq] = 0.0;
          }
        else if (fabs(a[ip][iq]) > tresh) 
          {
          h = w[iq] - w[ip];
          if ( (fabs(h)+g) == fabs(h))
            {
            t = (a[ip][iq]) / h;
            }
          else 
            {
            theta = 0.5*h / (a[ip][iq]);
            t = 1.0 / (fabs(theta)+sqrt(1.0+theta*theta));
            if (theta < 0.0)
              {
              t = -t;
              }
            }
          c = 1.0 / sqrt(1+t*t);
          s = t*c;
          tau = s/(1.0+c);
          h = t*a[ip][iq];
          z[ip] -= h;
          z[iq] += h;
          w[ip] -= h;
          w[iq] += h;
          a[ip][iq]=0.0;

          // ip already shifted left by 1 unit
          for (j = 0;j <= ip-1;j++) 
            {
            VTK_ROTATE(a,j,ip,j,iq);
            }
          // ip and iq already shifted left by 1 unit
          for (j = ip+1;j <= iq-1;j++) 
            {
            VTK_ROTATE(a,ip,j,j,iq);
            }
          // iq already shifted left by 1 unit
          for (j=iq+1; j<n; j++) 
            {
            VTK_ROTATE(a,ip,j,iq,j);
            }
          for (j=0; j<n; j++) 
            {
            VTK_ROTATE(v,j,ip,j,iq);
            }
          }
        }
      }

    for (ip=0; ip<n; ip++) 
      {
      b[ip] += z[ip];
      w[ip] = b[ip];
      z[ip] = 0.0;
      }
    }

  //// this is NEVER called
  if ( i >= 20 )
    {
    printf(
       "vtkMath::Jacobi: Error extracting eigenfunctions");
    return 0;
    }

  // sort eigenfunctions                 these changes do not affect accuracy 
  for (j=0; j<n-1; j++)                  // boundary incorrect
    {
    k = j;
    tmp = w[k];
    for (i=j+1; i<n; i++)                // boundary incorrect, shifted already
      {
      if (w[i] >= tmp)                   // why exchage if same?
        {
        k = i;
        tmp = w[k];
        }
      }
    if (k != j) 
      {
      w[k] = w[j];
      w[j] = tmp;
      for (i=0; i<n; i++) 
        {
        tmp = v[i][j];
        v[i][j] = v[i][k];
        v[i][k] = tmp;
        }
      }
    }
  // insure eigenvector consistency (i.e., Jacobi can compute vectors that
  // are negative of one another (.707,.707,0) and (-.707,-.707,0). This can
  // reek havoc in hyperstreamline/other stuff. We will select the most
  // positive eigenvector.
  int ceil_half_n = (n >> 1) + (n & 1);
  for (j=0; j<n; j++)
    {
    for (numPos=0, i=0; i<n; i++)
      {
      if ( v[i][j] >= 0.0 )
        {
        numPos++;
        }
      }
//    if ( numPos < ceil(double(n)/double(2.0)) )
    if ( numPos < ceil_half_n)
      {
      for(i=0; i<n; i++)
        {
        v[i][j] *= -1.0;
        }
      }
    }

  if (n > 4)
    {
    delete [] b;
    delete [] z;
    }
  return 1;
}

//----------------------------------------------------------------------------
// Find unit vectors which is perpendicular to this on and to
// each other.
void Perpendiculars(const CvPoint3D32f xx, CvPoint3D32f yy, double z[3],
                             double theta)
{
	double x[3]; x[0] = xx.x; x[1] = xx.y; x[2] = xx.z; 
	double y[3]; y[0] = yy.x; y[1] = yy.y; y[2] = yy.z; 

  int dx,dy,dz;
  double x2 = x[0]*x[0];
  double y2 = x[1]*x[1];
  double z2 = x[2]*x[2];
  double r = sqrt(x2 + y2 + z2);

  // transpose the vector to avoid divide-by-zero error
  if (x2 > y2 && x2 > z2)
  {
    dx = 0; dy = 1; dz = 2;
  }
  else if (y2 > z2) 
  {
    dx = 1; dy = 2; dz = 0;
  }
  else 
  {
    dx = 2; dy = 0; dz = 1;
  }

  double a = x[dx]/r;
  double b = x[dy]/r;
  double c = x[dz]/r;

  double tmp = sqrt(a*a+c*c);

  if (theta != 0)
    {
    double sintheta = sin(theta);
    double costheta = cos(theta);

//    if (y)
    if (true)
    {
      y[dx] = (c*costheta - a*b*sintheta)/tmp;
      y[dy] = sintheta*tmp;
      y[dz] = (-a*costheta - b*c*sintheta)/tmp;
    }

    if (z)
      {
      z[dx] = (-c*sintheta - a*b*costheta)/tmp;
      z[dy] = costheta*tmp;
      z[dz] = (a*sintheta - b*c*costheta)/tmp;
      }
    }
  else
    {
	//    if (y)
	  if (true)
    {
      y[dx] = c/tmp;
      y[dy] = 0;
      y[dz] = -a/tmp;
    }

    if (z)
      {
      z[dx] = -a*b/tmp;
      z[dy] = tmp;
      z[dz] = -b*c/tmp;
      }
    }      
}

CvMat* findTransform(vector<CvPoint3D32f> source, vector<CvPoint3D32f> target)
{
  
	CvMat *retMat = cvCreateMat(4,4,CV_32FC1);
	cvSetIdentity(retMat);
  // --- compute the necessary transform to match the two sets of landmarks ---

  /*
    The solution is based on
    Berthold K. P. Horn (1987),
    "Closed-form solution of absolute orientation using unit quaternions,"
    Journal of the Optical Society of America A, 4:629-642
  */

  // Original python implementation by David G. Gobbi

  const int N_PTS = source.size();
  if(N_PTS != target.size())
    {
    printf("Update: Source and Target Landmarks contain a different number of points\n");
    return retMat;
    }

  // -- if no points, stop here

  if (N_PTS == 0)
    {
    return retMat;
    }

  // -- find the centroid of each set --

  CvPoint3D32f source_centroid=cvPoint3D32f(0,0,0);
  CvPoint3D32f target_centroid=cvPoint3D32f(0,0,0);
  CvPoint3D32f p;
  for(int i=0;i<N_PTS;i++)
    {
    p = source.at(i);
    source_centroid.x += p.x;
    source_centroid.y += p.y;
    source_centroid.z += p.z;
    p = target.at(i);
    target_centroid.x += p.x;
    target_centroid.y += p.y;
    target_centroid.z += p.z;
    }
  source_centroid.x /= N_PTS;
  source_centroid.y /= N_PTS;
  source_centroid.z /= N_PTS;
  target_centroid.x /= N_PTS;
  target_centroid.y /= N_PTS;
  target_centroid.z /= N_PTS;

  // -- if only one point, stop right here

  if (N_PTS == 1)
    {
    CV_MAT_ELEM(*retMat, float, 0, 3) = target_centroid.x - source_centroid.x;
    CV_MAT_ELEM(*retMat, float, 1, 3) = target_centroid.y - source_centroid.y;
    CV_MAT_ELEM(*retMat, float, 2, 3) = target_centroid.z - source_centroid.z;
    return retMat;
    }

  // -- build the 3x3 matrix M --

  float M[3][3];
  float AAT[3][3];
  for(int i=0;i<3;i++)
    {
    AAT[i][0] = M[i][0]=0.0F; // fill M with zeros
    AAT[i][1] = M[i][1]=0.0F;
    AAT[i][2] = M[i][2]=0.0F;
    }

  float sa=0.0F,sb=0.0F;
  
  for(int pt=0;pt<N_PTS;pt++)
    {
		CvPoint3D32f a,b;
		// get the origin-centred point (a) in the source set
		a = source.at(pt);
	    a.x -= source_centroid.x;
		a.y -= source_centroid.y;
		a.z -= source_centroid.z;
		// get the origin-centred point (b) in the target set
		b = target.at(pt);
		b.x -= target_centroid.x;
		b.y -= target_centroid.y;
		b.z -= target_centroid.z;

		// accumulate the products a*T(b) into the matrix M
		M[0][0] += a.x*b.x;	M[0][1] += a.x*b.y; M[0][2] += a.x*b.z;
		M[1][0] += a.y*b.x;	M[1][1] += a.y*b.y;	M[1][2] += a.y*b.z;
		M[2][0] += a.z*b.x;	M[2][1] += a.z*b.y;	M[2][2] += a.z*b.z;

		// for the affine transform, compute ((a.a^t)^-1 . a.b^t)^t.
		// a.b^t is already in M.  here we put a.a^t in AAT.
/*		if (this->Mode == VTK_LANDMARK_AFFINE)
		{
			AAT[0][0] += a.x*a.x;	AAT[0][1] += a.x*a.y;	AAT[0][2] += a.x*a.z;
			AAT[1][0] += a.y*a.x;	AAT[1][1] += a.y*a.y;	AAT[1][2] += a.y*a.z;
			AAT[2][0] += a.z*a.x;	AAT[2][1] += a.z*a.y;	AAT[2][2] += a.z*a.z;
		}
*/

	// accumulate scale factors (if desired)
    sa += a.x*a.x+a.y*a.y+a.z*a.z;
    sb += b.x*b.x+b.y*b.y+b.z*b.z;
  }

/*  if(this->Mode == VTK_LANDMARK_AFFINE)
    {
    // AAT = (a.a^t)^-1
    vtkMath::Invert3x3(AAT,AAT);

    // M = (a.a^t)^-1 . a.b^t
    vtkMath::Multiply3x3(AAT,M,M);

    // this->Matrix = M^t
    for(int i=0;i<3;++i)
      {
      for(int j=0;j<3;++j)
	{
	this->Matrix->Element[i][j] = M[j][i];
	}
      }
    }
  else*/
    {
    // compute required scaling factor (if desired)
    float scale = (float)sqrt(sb/sa);

    // -- build the 4x4 matrix N --

    float Ndata[4][4];
    float *N[4];
    for(int i=0;i<4;i++)
      {
      N[i] = Ndata[i];
      N[i][0]=0.0F; // fill N with zeros
      N[i][1]=0.0F;
      N[i][2]=0.0F;
      N[i][3]=0.0F;
      }
    // on-diagonal elements
    N[0][0] = M[0][0]+M[1][1]+M[2][2];
    N[1][1] = M[0][0]-M[1][1]-M[2][2];
    N[2][2] = -M[0][0]+M[1][1]-M[2][2];
    N[3][3] = -M[0][0]-M[1][1]+M[2][2];
    // off-diagonal elements
    N[0][1] = N[1][0] = M[1][2]-M[2][1];
    N[0][2] = N[2][0] = M[2][0]-M[0][2];
    N[0][3] = N[3][0] = M[0][1]-M[1][0];

    N[1][2] = N[2][1] = M[0][1]+M[1][0];
    N[1][3] = N[3][1] = M[2][0]+M[0][2];
    N[2][3] = N[3][2] = M[1][2]+M[2][1];

    // -- eigen-decompose N (is symmetric) --

    float eigenvectorData[4][4];
    float *eigenvectors[4],eigenvalues[4];

    eigenvectors[0] = eigenvectorData[0];
    eigenvectors[1] = eigenvectorData[1];
    eigenvectors[2] = eigenvectorData[2];
    eigenvectors[3] = eigenvectorData[3];

    JacobiN(N,4,eigenvalues,eigenvectors);

    // the eigenvector with the largest eigenvalue is the quaternion we want
    // (they are sorted in decreasing order for us by JacobiN)
    double w,x,y,z;

    // first: if points are collinear, choose the quaternion that
    // results in the smallest rotation.
    if (eigenvalues[0] == eigenvalues[1] || N_PTS == 2)
      {
		CvPoint3D32f s0,t0,s1,t1;
		s0 = source.at(0);
		t0 = target.at(0);
		s1 = source.at(1);
		t1 = target.at(1);

      CvPoint3D32f ds,dt;
      double rs = 0, rt = 0;

		ds.x = s1.x - s0.x;      // vector between points
		rs += ds.x*ds.x;
		dt.x = t1.x - t0.x;
		rt += dt.x*dt.x;

		ds.y = s1.y - s0.y;      // vector between points
		rs += ds.y*ds.y;
		dt.y = t1.y - t0.y;
		rt += dt.y*dt.y;

		ds.z = s1.z - s0.z;      // vector between points
		rs += ds.z*ds.z;
		dt.z = t1.z - t0.z;
		rt += dt.z*dt.z;

      // normalize the two vectors
      rs = sqrt(rs);
      ds.x /= rs; ds.y /= rs; ds.z /= rs;
      rt = sqrt(rt);
      dt.x /= rt; dt.y /= rt; dt.z /= rt;

      // take dot & cross product
      w = ds.x*dt.x + ds.y*dt.y + ds.z*dt.z;
      x = ds.y*dt.z - ds.z*dt.y;
      y = ds.z*dt.x - ds.x*dt.z;
      z = ds.x*dt.y - ds.y*dt.x;

      double r = sqrt(x*x + y*y + z*z);
      double theta = atan2(r,w);

      // construct quaternion
      w = cos(theta/2);
      if (r != 0)
	{
	r = sin(theta/2)/r;
	x = x*r;
	y = y*r;
	z = z*r;
	}
      else // rotation by 180 degrees: special case
	{
	// rotate around a vector perpendicular to ds
	Perpendiculars(ds,dt,0,0);
	r = sin(theta/2);
	x = dt.x*r;
	y = dt.y*r;
	z = dt.z*r;
	}
      }
    else // points are not collinear
      {
      w = eigenvectors[0][0];
      x = eigenvectors[1][0];
      y = eigenvectors[2][0];
      z = eigenvectors[3][0];
      }

    // convert quaternion to a rotation matrix

    double ww = w*w;
    double wx = w*x;
    double wy = w*y;
    double wz = w*z;

    double xx = x*x;
    double yy = y*y;
    double zz = z*z;

    double xy = x*y;
    double xz = x*z;
    double yz = y*z;

    CV_MAT_ELEM(*retMat, float, 0, 0) = ww + xx - yy - zz;
    CV_MAT_ELEM(*retMat, float, 1, 0) = 2.0*(wz + xy);
    CV_MAT_ELEM(*retMat, float, 2, 0) = 2.0*(-wy + xz);

    CV_MAT_ELEM(*retMat, float, 0, 1) = 2.0*(-wz + xy);
    CV_MAT_ELEM(*retMat, float, 1, 1) = ww - xx + yy - zz;
    CV_MAT_ELEM(*retMat, float, 2, 1) = 2.0*(wx + yz);

    CV_MAT_ELEM(*retMat, float, 0, 2) = 2.0*(wy + xz);
    CV_MAT_ELEM(*retMat, float, 1, 2) = 2.0*(-wx + yz);
    CV_MAT_ELEM(*retMat, float, 2, 2) = ww - xx - yy + zz;

//    if (this->Mode != VTK_LANDMARK_RIGIDBODY)
      { // add in the scale factor (if desired)
      for(int i=0;i<3;i++)
	{
	CV_MAT_ELEM(*retMat, float, i, 0) *= scale;
	CV_MAT_ELEM(*retMat, float, i, 1) *= scale;
	CV_MAT_ELEM(*retMat, float, i, 2) *= scale;
	}
      }
    }

  // the translation is given by the difference in the transformed source
  // centroid and the target centroid
  double sx, sy, sz;

  sx = CV_MAT_ELEM(*retMat, float, 0, 0) * source_centroid.x +
       CV_MAT_ELEM(*retMat, float, 0, 1) * source_centroid.y +
       CV_MAT_ELEM(*retMat, float, 0, 2) * source_centroid.z;
  sy = CV_MAT_ELEM(*retMat, float, 1, 0) * source_centroid.x +
       CV_MAT_ELEM(*retMat, float, 1, 1) * source_centroid.y +
       CV_MAT_ELEM(*retMat, float, 1, 2) * source_centroid.z;
  sz = CV_MAT_ELEM(*retMat, float, 2, 0) * source_centroid.x +
       CV_MAT_ELEM(*retMat, float, 2, 1) * source_centroid.y +
       CV_MAT_ELEM(*retMat, float, 2, 2) * source_centroid.z;

  CV_MAT_ELEM(*retMat, float, 0, 3) = target_centroid.x - sx;
  CV_MAT_ELEM(*retMat, float, 1, 3) = target_centroid.y - sy;
  CV_MAT_ELEM(*retMat, float, 2, 3) = target_centroid.z - sz;

  // fill the bottom row of the 4x4 matrix
  CV_MAT_ELEM(*retMat, float, 3, 0) = 0.0;
  CV_MAT_ELEM(*retMat, float, 3, 1) = 0.0;
  CV_MAT_ELEM(*retMat, float, 3, 2) = 0.0;
  CV_MAT_ELEM(*retMat, float, 3, 3) = 1.0;

  return retMat;
}

#endif
