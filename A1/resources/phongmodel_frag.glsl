#version 410

in vec3 vPos; // in camera space
in vec3 vNor; // in camera space
in vec2 vTex;
uniform vec3 lightPos; // in camera space
uniform vec3 ka;
uniform vec3 ks;
uniform float s;
uniform sampler2D kdTex;

out vec4 FragColor;

void main()
{
	vec3 n = normalize(vNor);
	vec3 l = normalize(lightPos - vPos);
	vec3 v = -normalize(vPos);
	vec3 h = normalize(l + v);
	vec4 colorT = texture(kdTex, vTex.st).rgba;
	vec3 colorA = ka;
	vec3 colorD = max(dot(l, n), 0.0) * colorT.rgb;
	vec3 colorS = pow(max(dot(h, n), 0.0), s) * ks;
	vec3 color = colorA + colorD + colorS;
	FragColor = vec4(color, colorT.a);
	//gl_FragColor = vec4(vTex.xy, 0.0, 1.0);
}
