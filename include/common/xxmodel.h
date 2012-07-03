#ifndef SUBMODEL20SIM_H
#define SUBMODEL20SIM_H

#include "xxtypes.h"
#include "xxmatrix.h"


class Submodel20sim
{
	protected:
		friend class IntegrationMethod;
		friend class Discrete;
		friend class Euler;
		friend class RungeKutta2;
		friend class RungeKutta4;
		virtual void CalculateDynamic (void) {};

		bool     %VARPREFIX%initialize;
		bool     %VARPREFIX%major;

	public:
		virtual ~Submodel20sim(){};
		
		XXDouble %VARPREFIX%step_size;
		XXDouble %VARPREFIX%start_time;
		XXDouble %VARPREFIX%finish_time;
		XXDouble %VARPREFIX%%XX_TIME%;

		/* the variable count */
		int %VARPREFIX%number_constants;
		int %VARPREFIX%number_parameters;
		int %VARPREFIX%number_initialvalues;
		int %VARPREFIX%number_variables;
		int %VARPREFIX%number_states;
		int %VARPREFIX%number_rates;
		int %VARPREFIX%number_matrices;
		int %VARPREFIX%number_unnamed;

		/* the variable arrays are allocated in the derived submodel class */
		XXDouble* %VARPREFIX%%XX_CONSTANT_ARRAY_NAME%;					/* constants */
		XXDouble* %VARPREFIX%%XX_PARAMETER_ARRAY_NAME%;					/* parameters */
		XXDouble* %VARPREFIX%%XX_INITIAL_VALUE_ARRAY_NAME%;				/* initial values */
		XXDouble* %VARPREFIX%%XX_VARIABLE_ARRAY_NAME%;					/* variables */
		XXDouble* %VARPREFIX%%XX_STATE_ARRAY_NAME%;						/* states */
		XXDouble* %VARPREFIX%%XX_RATE_ARRAY_NAME%;						/* rates (or new states) */
		XXMatrix* %VARPREFIX%%XX_MATRIX_ARRAY_NAME%;					/* matrices */
		XXDouble* %VARPREFIX%%XX_UNNAMED_ARRAY_NAME%;					/* unnamed */
		XXDouble* %VARPREFIX%workarray;
};

#endif 	// SUBMODEL20SIM_H
