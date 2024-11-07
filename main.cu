#define N 8

__global__ void vec_add(float *A, float *B, float *C)
{
	int i = threadIdx.x;
	C[i] = A[i] + B[i];
}

int main()
{
	float A[N] = {1, 2, 3, 4, 5, 6, 7, 8};
	float B[N] = {8, 7, 6, 5, 4, 3, 2, 1};
	float C[N] = {};

	vec_add<<<1, N>>>(A, B, C);

	printf("hello world");

	return 0;
}
