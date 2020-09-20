#include "../ray_tracing.h"

int main() {
	ComponentParam componet;
	Antops::RayTracingOption option;
	option.algo_type = Antops::AnalysisType;
	Antops::RayTracing ray_tracing(option);
	
	return 0;
}