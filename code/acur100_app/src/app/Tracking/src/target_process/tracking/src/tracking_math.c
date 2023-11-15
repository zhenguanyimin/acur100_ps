
#include <math.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"


/**
*  @b Description
*  @n
*		This function is used to create identity matrix 
*
*  @param[in]  size
*		Size of identity matrix
*  @param[out] A
*		Matrix A(size,size) = eye(size)
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixEye(uint16_t size, float *A)
{
	/* A(size*size) = eye(size) */
	uint16_t i;

	for (i = 0U; i < size*size; i++)
	{
		A[i] = 0.f;
	}

	for (i = 0U; i < size; i++)
	{
        A[i+i*size] = 1.0f;
	}
}

/**
*  @b Description
*  @n
*		This function is used to create identity matrix 
*
*  @param[in]  size
*		Size of identity matrix
*  @param[out] A
*		Matrix A(size*(size-1)/2) = triangleeye(size)
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixTriangleEye(uint16_t size, float *A)
{
	uint16_t i, j, num = 0;
	for (i = 0U; i < size; i++)
	{
		for(j = i; j < size; j++)
		{
			if(j == i)
			{
				A[num] = 1.0f;
			}
			else
			{
				A[num] = 0.f;
			}
			num++;
		}
	}
}
/**
*  @b Description
*  @n
*		This function is used to initialise matrix to a value 
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  value
*		value to set
*  @param[out] A
*		Matrix A(rows,cols) = ones(rows,cols)*value
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixInit(uint16_t rows, uint16_t cols, float value, float *A)
{
	/* A(rows*cols) = ones(rows,cols)*value */
	uint16_t i;

	for (i = 0U; i < rows*cols; i++)
	{
		A[i] = value;
	}
}

/**
*  @b Description
*  @n
*		This function is used to create diagonal square matrix 
*
*  @param[in]  size
*		Size of square matrix
*  @param[in]  v
*		Diagonal vector
*  @param[out] A
*		Matrix A(size,size) = diag(v(size))
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixSetDiag(uint16_t size, float *v, float *A)
{
	/* A(size*size) = diag(v(size)) */
	uint16_t i;

	for (i = 0U; i < size*size; i++)
	{
		A[i] = 0;
	}

	for (i = 0U; i < size; i++)
	{
        A[i+i*size] = v[i];
	}
}

/**
*  @b Description
*  @n
*		This function is used to get diagonal from the square matrix 
*
*  @param[in]  size
*		Size of square matrix
*  @param[in] A
*		Matrix A(size,size)
*  @param[out]  v
*		Diagonal vector, v(size) = diag(A(size*size))
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixGetDiag(uint16_t size, float *A, float *v)
{
	/* v(size) = diag(A(size*size)) */
	uint16_t i;
	for (i = 0U; i < size; i++)
	{
        v[i] = A[i+i*size];
	}
}

/**
*  @b Description
*  @n
*		This function is used to multiply two matrices.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Outer dimension, number of rows
*  @param[in]  m
*		Inner dimension
*  @param[in]  cols
*		Outer dimension, number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[out]  C
*		Matrix C(rows,cols) = A(rows,m) X B(cols,m)T
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixMultiply(uint16_t rows, uint16_t m, uint16_t cols, float *A, float *B, float *C)
{
	/* C(rows*cols) = A(rows*m)*B(m*cols) */
	uint16_t i,j, k;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols; j++)
		{			
			C[i*cols + j] = 0;
			for (k = 0; k < m; k++)
			{
				C[i*cols+j] += A[i*m+k] * B[k*cols + j];
			}
		}
	}
}

/**
*  @b Description
*  @n
*		This function is used to multiply two matrices. Second Matrix is getting transposed first
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Outer dimension, number of rows
*  @param[in]  m
*		Inner dimension
*  @param[in]  cols
*		Outer dimension, number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[out]  C
*		Matrix C(rows,cols) = A(rows,m) X B(cols,m)T
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixTransposeMultiply(uint16_t rows, uint16_t m, uint16_t cols, float *A, float *B, float *C)
{
	/* C(rows*cols) = A(rows*m)*B(cols*m)T */
	uint16_t i,j, k;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols; j++)
		{			
			C[i*cols + j] = 0;
			for (k = 0; k < m; k++)
			{
				C[i*cols+j] += A[i*m+k] * B[k + j*m];
			}
		}
	}
}

/**
*  @b Description
*  @n
*		This function is used to multiply matrix by a scalar.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  c
*		Scalar c
*  @param[out]  B
*		Matrix B(rows,cols) = A(rows,cols) * c
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixScalarMultiply(uint16_t rows, uint16_t cols, float *A, float c, float *B)
{
	/* B(rows*cols) = A(rows*cols)*C */
	uint16_t i;
	for (i = 0U; i < rows*cols; i++)
	{
		B[i] = A[i] * c;
	}
}

/**
*  @b Description
*  @n
*		This function is used to add two matrices.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[out]  C
*		Matrix C(rows,cols) = A(rows,cols) + B(rows,cols)
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixAdd(uint16_t rows, uint16_t cols, float *A, float *B, float *C)
{
	/* C(rows*cols) = A(rows*cols) + B(rows*cols) */
	uint16_t i;
	for (i = 0U; i < rows*cols; i++)
	{
		C[i] = A[i] + B[i];
	}
}

/**
*  @b Description
*  @n
*		This function is used to subtract two matrices.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[out]  C
*		Matrix C(rows,cols) = A(rows,cols) - B(rows,cols)
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixSub(uint16_t rows, uint16_t cols, float *A, float *B, float *C)
{
	/* C(rows*cols) = A(rows*cols) - B(rows*cols) */
	uint16_t i;
	for (i = 0U; i < rows*cols; i++)
	{
		C[i] = A[i] - B[i];
	}
}

/**
*  @b Description
*  @n
*		This function is used to force matrix symmetry by averaging off-diagonal elements
*		Matrices are squared, real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  m (m=rows=cols)
*		Number of rows and cols
*  @param[in]  A
*		Matrix A
*  @param[out]  B
*		Matrix B
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixMakeSymmetrical(uint16_t m, float *A, float *B)
{
	/* Make square matrix symmetrical by averaging off-diagonal elements */
	uint16_t i,j;
	for (i = 0U; i < m-1; i++)
	{
		B[i*m + i] = A[i*m + i];
		for (j = i+1; j < m; j++)
		{
			B[i*m+j] = B[j*m+i] = 0.5f*(A[i*m+j]+A[j*m+i]);
		}
	}
	B[i*m + i] = A[i*m + i];
}

/**
*  @b Description
*  @n
*		This function performs cholesky decomposition of square matrix.
*		Matrix are squared, real, single precision floating point.
*		Matrix are in row-major order
*
*  @param[in]  dim (dim=rows=cols)
*		Number of rows and cols
*  @param[in]  A
*		Matrix A
*  @param[out]  G
*		Matrix G = cholseky(A);
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixCholesky(uint16_t dim, const float *A, float *G)
{
    float v[4] = {0.f};
	float temp;
    uint16_t i, j, k;

    for (i = 0U; i < dim*dim; i++)
	{
        G[i] = 0;
	}

    for (j = 0U; j < dim; j++)
    {
		/* v(j:n,1) = A(j:n,j) */
		for (i = j; i < dim; i++)
		{
		    v[i] = A[i * dim + j];
        }
		
		for (k = 0U; k < j; k++)
        {
			/* v(j:n,1) = v(j:n,1) - G(j,k)*G(j:n,k) */
			for (i = j; i < dim; i++)
			{
				v[i] = v[i] - G[j*dim + k] * G[i*dim + k];
			}
		}
    
        /* G(j:n,j) = v(j:n,1)/sqrt(v(j)) */
		temp = 1.0f/sqrtf(v[j]);
		for (i = j; i < dim; i++)
		{
			G[i*dim + j] = v[i]*temp;
		}
	}
}

/**
*  @b Description
*  @n
*		This function is used to initialise vector to a value 
*
*  @param[in]  size
*		Size of vector
*  @param[in]  value
*		value to set
*  @param[out]  A
*		Vector A
*		
*       A(size) = ones(size,1)*value
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_vectorInit(uint16_t size, float value, float *A)
{
	/* A(size) = ones(size,1)*value */
	uint16_t i;

	for (i = 0U; i < size; i++)
	{
		A[i] = value;
	}
}
/**
*  @b Description
*  @n
*		This function adds two vectors
*		Vectors are real, single precision floating point.
*
*  @param[in]  size
*		Size of vector
*  @param[in]  A
*		Vector A
*  @param[in]  B
*		Vector B
*  @param[out]  C
*		Vector C = A + B;
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_vectorAdd(uint16_t size, float *A, float *B, float *C)
{
	uint16_t i;
	for (i = 0U; i < size; i++)
	{
        C[i] = A[i] + B[i];
	}
}

/**
*  @b Description
*  @n
*		This function subtracts two vectors
*		Vectors are real, single precision floating point.
*
*  @param[in]  size
*		Size of vectors
*  @param[in]  A
*		Vector A
*  @param[in]  B
*		Vector B
*  @param[out]  C
*		Vector C = A - B;
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_vectorSub(uint16_t size, float *A, float *B, float *C)
{
	uint16_t i;
	for (i = 0U; i < size; i++)
	{
        C[i] = A[i] - B[i];
	}
}

/**
*  @b Description
*  @n
*		This function multiplies vector by scalar
*		Vectors are real, single precision floating point.
*		Scalar is real, single precision floating point.
*
*  @param[in]  size
*		Size of vector
*  @param[in]  A
*		Vector A
*  @param[in]  c
*		Scalar c
*  @param[out]  B
*		Vector B = A*c;
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_vectorScalarMul(uint16_t size, float *A, float c, float *B)
{	
    uint16_t i;
	for (i = 0U; i < size; i++)
	{
        B[i] = A[i]*c;
	}
}

/**
*  @b Description
*  @n
*		This function performs multiplies vector by scalar and accumulates the results
*		Vectors are real, single precision floating point.
*		Scalar is real, single precision floating point.
*
*  @param[in]  size
*		Size of vector
*  @param[in]  A
*		Vector A
*  @param[in]  c
*		Scalar c
*  @param[in, out]  B
*		Vector B = B + A*c;
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_vectorScalarMulAcc(uint16_t size, float *A, float c, float *B)
{	
    uint16_t i;
	for (i = 0U; i < size; i++)
	{
        B[i] = B[i] + A[i]*c;
	}
}

/**
*  @b Description
*  @n
*		This function performs IIR vector filtering
*		Vectors are real, single precision floating point.
*		Alpha is real, single precision floating point.
*
*  @param[in]  size
*		Size of vector
*  @param[in, out]  A
*		Vector A
*  @param[in]  alpha
*		Weighting factor for new information, (0<=alpha<=1.0f)
*  @param[in]  B
*		New information vector B
*
*		Vector A = A*(1.0f-alpha) + B*alpha;
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_vectorFilter(uint16_t size, float *A, float alpha, float *B)
{	
    uint16_t i;
	for (i = 0U; i < size; i++)
	{
        A[i] = A[i]*(1.0f - alpha) + B[i]*alpha;
	}
}


/**
*  @b Description
*  @n
*		This function accumulates covariance matrix with variances from input vector and mean
*		Matrices are all real, single precision floating point.
*		Vectors are real, single precision floating point.
*
*  @param[in]  size
*		Size of square matrix
*  @param[in]  A
*		Matrix A
*  @param[in]  v
*		Vector v
*  @param[in]  mean
*		Vector mean
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixCovAcc(uint16_t size, float *A, float *v, float *mean)
{
	uint16_t i,j;
    float d1, d2;

	for (i = 0U; i < size; i++) 
	{
        d1 = v[i]-mean[i];
        for (j = i; j < size; j++) 
		{
            d2 = v[j]-mean[j];
            A[i*size+j] += d1*d2;
        }
    }
}

/**
*  @b Description
*  @n
*		This function normalizes covariance matrix
*		Matrices are all real, single precision floating point.
*
*  @param[in]  size
*		Size of square matrix
*  @param[in,out]  A
*		Matrix A
*  @param[in]  num
*		Number of measurments num
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixCovNormalize(uint16_t size, float *A, uint16_t num)
{
	uint16_t i,j;
	for (i = 0U; i < size; i++) 
	{
		num = (num==0)? 1 : num;
        A[i*size+i] /= num;
        for (j = i+1; j < size; j++) 
		{
            A[i*size+j] /= num;
            A[i+j*size] = A[i*size+j];
        }
    }
}        
/**
*  @b Description
*  @n
*		This function filters covariance matrix
*		Matrices are all real, single precision floating point.
*
*  @param[in]  size
*		Size of square matrix
*  @param[in,out]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[in]  alpha
*		Filtering coefficient alpha
*  Matrix A = (1-alpha)*A + alpha*B
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void tracking_matrixCovFilter(uint16_t size, float *A, float *B, float alpha)
{
	uint16_t i,j;
	for (i = 0U; i < size; i++) 
	{
        A[i*size+i] = (1-alpha)*A[i*size+i] + alpha*B[i*size+i];
        for (j = i+1; j < size; j++) 
		{
            A[i*size+j] = (1-alpha)*A[i*size+j] + alpha*B[i*size+j];
            A[i+j*size] = A[i*size+j];
        }
    }
}        

/**
*  @b Description
*  @n
*		This function prints a matrix
*		Matrices are all real, single precision floating point.
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  A
*		Matrix A
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixPrint(uint16_t rows, uint16_t cols, float *A)
{
	uint16_t i,j;
	for (i = 0U; i < rows; i++)
	{
		for (j = 0U; j < cols-1; j++)
		{
#ifdef DEBUG_LOG
			//my_printf("%6.4f\t", A[i*cols +j]); 
#endif
		}
#ifdef DEBUG_LOG
		//my_printf("%6.4f\t\n", A[i*cols +j]); 
#endif
	}
#ifdef DEBUG_LOG
	//my_printf("\n"); 
#endif
}
void tracking_matrixPrint_double(uint16_t rows, uint16_t cols, double *A)
{
	uint16_t i,j;
	for (i = 0U; i < rows; i++)
	{
		for (j = 0U; j < cols-1; j++)
		{
#ifdef DEBUG_LOG
			//my_printf("%6.4f ", A[i*cols +j]); 
#endif
		}
#ifdef DEBUG_LOG
		//my_printf("%6.4f \n", A[i*cols +j]); 
#endif
	}
#ifdef DEBUG_LOG
	//my_printf("\n"); 
#endif
}
void tracking_matrixPrint_int(uint16_t rows, uint16_t cols, int *A)
{
	uint16_t i,j;
	for (i = 0U; i < rows; i++)
	{
		for (j = 0U; j < cols-1; j++)
		{
#ifdef DEBUG_LOG
			//my_printf("%d ", A[i*cols +j]); 
#endif
		}
#ifdef DEBUG_LOG
		//my_printf("%d \n", A[i*cols +j]); 
#endif
	}
#ifdef DEBUG_LOG
	//my_printf("\n"); 
#endif
}
/**
*  @b Description
*  @n
*		This function prints 2 matrices side by side
*		Matrices are all real, single precision floating point.
*
*  @param[in]  rows
*		Number of rows
*  @param[in]  cols
*		Number of cols
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*
*  \ingroup TRACKING_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixPrint2(uint16_t rows, uint16_t cols, float *A, float *B)
{

	/* print 2 matrices side by side */
	uint16_t i,j;
	for (i = 0U; i < rows; i++)
	{
		for (j = 0U; j < cols-1; j++)
		{
#ifdef DEBUG_LOG
			//my_printf("%6.4f\t", A[i*cols +j]); 
#endif
		}
		//tracking_log(TRACKING_VERBOSE_DEBUG, "%6.2f\t\t",A[i*cols +j]);
#ifdef DEBUG_LOG
		//my_printf("%6.4f\t\t", A[i*cols +j]); 
#endif
		for (j = 0U; j < cols-1; j++)
		{
#ifdef DEBUG_LOG
			//my_printf("%6.4f\t\t", B[i*cols +j]); 
#endif
		}
		//tracking_log(TRACKING_VERBOSE_DEBUG, "%6.2f\n",B[i*cols +j]);
#ifdef DEBUG_LOG
		//my_printf("%6.4f\t\n", B[i*cols +j]); 
#endif
	}
#ifdef DEBUG_LOG
	//my_printf("\n"); 
#endif
}
