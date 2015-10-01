#version 330 core

uniform mat4 ModelViewMatrix;
uniform vec4 lightPosition;
uniform vec4 ambientLight;
uniform vec4 diffuseLight;
uniform vec4 specularLight;
uniform sampler2DShadow shadowMap;

in vec4 colorV;
in vec3 fragNormal;
in vec4 fragVert;
in vec4 shadowCoord;

layout(location = 0) out vec4 fragColor;

vec2 poissonDisk[16] = vec2[]( 
   vec2( -0.94201624, -0.39906216 ), 
   vec2( 0.94558609, -0.76890725 ), 
   vec2( -0.094184101, -0.92938870 ), 
   vec2( 0.34495938, 0.29387760 ), 
   vec2( -0.91588581, 0.45771432 ), 
   vec2( -0.81544232, -0.87912464 ), 
   vec2( -0.38277543, 0.27676845 ), 
   vec2( 0.97484398, 0.75648379 ), 
   vec2( 0.44323325, -0.97511554 ), 
   vec2( 0.53742981, -0.47373420 ), 
   vec2( -0.26496911, -0.41893023 ), 
   vec2( 0.79197514, 0.19090188 ), 
   vec2( -0.24188840, 0.99706507 ), 
   vec2( -0.81409955, 0.91437590 ), 
   vec2( 0.19984126, 0.78641367 ), 
   vec2( 0.14383161, -0.14100790 ) 
);

void main(void)
{
	mat3 normalMatrix = transpose(inverse(mat3(ModelViewMatrix)));
	vec3 n = normalize(normalMatrix * fragNormal);

	vec3 fragPosition = vec3(ModelViewMatrix * fragVert);
	vec3 fLight = normalize(vec3(lightPosition) - fragPosition);
	float kD = max(dot(fLight, n), 0);

	vec3 N = reflect(-fLight, n);
	vec3 V = normalize(-fragPosition);
	float kS = pow(max(dot(N, V), 0), 16.0); 

//	float attenuation = 1.0 / (1.0 + 5e-7 * pow(length(vec3(cameraPosition) - fragPosition), 2));

	float visibility = 1.0;
	float bias = 0.005;
	for (int i = 0; i < 4; i++) {
		visibility -= 0.2 * (1.0 - texture( shadowMap, vec3(shadowCoord.xy + poissonDisk[i] / 700.0f, (shadowCoord.z - bias) / shadowCoord.w)));
	}

    fragColor.rgb = visibility * kD * vec3(diffuseLight * colorV) + vec3(ambientLight * colorV) + visibility * kS * vec3(specularLight * colorV);
	fragColor.a = colorV.a;
}