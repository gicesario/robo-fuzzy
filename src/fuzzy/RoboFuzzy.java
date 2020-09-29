package fuzzy;

import java.io.File;

import coppelia.FloatWA;
import coppelia.remoteApi;
import net.sourceforge.jFuzzyLogic.FIS;
import net.sourceforge.jFuzzyLogic.FunctionBlock;

public class RoboFuzzy {
	
    public static boolean encontrou_alvo = false;

    @SuppressWarnings("static-access")
	public static void main(String[] args) throws Exception {
        System.out.println("Inicio...");
        
        remoteApi vrep = new remoteApi();
        
        vrep.simxFinish(-1); 
        
        int clientID = vrep.simxStart("127.0.0.1", 19999, true, true, 5000, 5); // socket v-rep
     
        if (clientID != -1) {
            System.out.println("Conexao efetuada.");   
        
            vrep.simxSynchronous(clientID,true);

            // Carrega os arquivos com as regras de inferencia
            String path = new File(RoboFuzzy.class.getProtectionDomain().getCodeSource().getLocation().toURI()).getPath();
            String filePath = path+File.separator+"fuzzy"+File.separator;
            String roboFile = filePath+"mover_robo.fcl";            
            String alvoFile = filePath+"buscar_alvo.fcl";

            FIS fisRobo = FIS.load(roboFile, true);
            FIS fisAlvo = FIS.load(alvoFile, true);
            
            if (fisRobo == null) {
            	System.err.println("Erro ao carregar arquivos ");
            	return;
            }

            // Obtem as regras parametrizadas:
            FunctionBlock roboFunctionBlock = fisRobo.getFunctionBlock(null);
            FunctionBlock alvoFunctionBlock = fisAlvo.getFunctionBlock(null);
            
        //    JFuzzyChart.get().chart(roboFunctionBlock);
          //  JFuzzyChart.get().chart(alvoFunctionBlock);
            
            FloatWA velocidadeMotores = new FloatWA(2);
            FloatWA sensores = new FloatWA(6);
            
            FloatWA roboDirecao = new FloatWA(3);
            FloatWA roboPosicao = new FloatWA(3);            
            FloatWA alvoDirecao = new FloatWA(3);

            while(!encontrou_alvo) {
                // Obtem sensores
                vrep.simxCallScriptFunction(clientID, "K3_robot", vrep.sim_scripttype_childscript, "getSensors", null, null, null, null, null, sensores, null, null, vrep.simx_opmode_blocking);
                
                // obtem a direcao do robo
                vrep.simxCallScriptFunction(clientID, "K3_robot", vrep.sim_scripttype_childscript, "getOrientation", null, null, null, null, null, roboDirecao, null, null, vrep.simx_opmode_blocking);
                float angleA = roboDirecao.getArray()[0];
                float angleB = roboDirecao.getArray()[1];
                float angle = ((angleA >= 0 ? 1f : -1f) * (float)Math.toRadians(90)) + angleB;
                double angle360 = Math.toDegrees(angle) > 0 ? Math.toDegrees(angle) : (Math.toDegrees(angle)*-1) + 180;
                
                // obtem a posicao do robo
                vrep.simxCallScriptFunction(clientID, "K3_robot", vrep.sim_scripttype_childscript, "getPosition", null, null, null, null, null, roboPosicao, null, null, vrep.simx_opmode_blocking);
                float robotX = roboPosicao.getArray()[0];
                float robotY = roboPosicao.getArray()[1];
                
                // obtem a posicao do alvo
                vrep.simxCallScriptFunction(clientID, "K3_robot", vrep.sim_scripttype_childscript, "getTargetPosition", null, null, null, null, null, alvoDirecao, null, null, vrep.simx_opmode_blocking);
                float targetX = alvoDirecao.getArray()[0];
                float targetY = alvoDirecao.getArray()[1];
                
                // calcula angulo do alvo
                float destino_distanciaX = robotX - targetX;
                float destino_distanciaY = robotY - targetY;
                double destino_distanciaToTarget = Math.sqrt(Math.pow(destino_distanciaX,2) + Math.pow(destino_distanciaY,2));
                double angleToTarget = Math.atan2(destino_distanciaY, destino_distanciaX);
                double angleToTarget360 = ((((Math.toDegrees(angleToTarget)-90)%360) + 360) % 360) ; // correcao para bater com o angulo do robo
                   
                float L = Math.min(sensores.getArray()[0], 0.2f) * 15;
                float LM = Math.min(sensores.getArray()[1], 0.2f) * 15;                
                float LF = Math.min(sensores.getArray()[2], 0.2f) * 15;
                float RF = Math.min(sensores.getArray()[3], 0.2f) * 15;
                float F = (LF + RF) / 2;                
                float RM = Math.min(sensores.getArray()[4], 0.2f) * 15;
                float R = Math.min(sensores.getArray()[5], 0.2f) * 15;                
                float motor_esquerdo = 0;
                float motor_direito = 0; 
                
                System.out.println("L  = " + L); 
                System.out.println("LM = " + LM); 
                System.out.println("F  = " + F); 
                System.out.println("RM = " + RM); 
                System.out.println("R  = " + R); 
                System.out.println("L+LM+F+RM+R = " + (L+LM+F+RM+R)); 
          
                // Desvia de qualquer obstaculo no caminho
                if(L+LM+F+RM+R < 15) {
            	
                    // Desvia dos obstaculos FUZZY
                	
                    // Set INPUTS
                    roboFunctionBlock.setVariable("esquerda", L);
                    roboFunctionBlock.setVariable("meia_esquerda", LM);
                    roboFunctionBlock.setVariable("frente", F);
                    roboFunctionBlock.setVariable("meia_direita", RM);
                    roboFunctionBlock.setVariable("direita", R);

                    // Evaluate 
                    roboFunctionBlock.evaluate();

                    // ajusta velocidade dos motores
                    motor_esquerdo = (float)(roboFunctionBlock.getVariable("motor_esquerdo").getValue() * 2);
                    motor_direito = (float)(roboFunctionBlock.getVariable("motor_direito").getValue() * 2);
                                         
                }
                else {
                    // Busca o alvo 
                	
                    // Calculos para calcular a diferenca dos angulos
                    float dif = (float)Math.toRadians(angleToTarget360 - ((((angle360-180)%360) + 360) % 360));
                    float diferencaAngulos = (float)Math.atan2(Math.sin(dif), Math.cos(dif));
                    float diferencaAjustada = (float)(Math.toDegrees(diferencaAngulos) / 36) * -1;
                    
                    System.out.println("diferencaAngulos  = " + diferencaAngulos); 
                    System.out.println("diferencaAjustada = " + diferencaAjustada);    
                    
                    alvoFunctionBlock.setVariable("destino_distancia", destino_distanciaToTarget);
                    alvoFunctionBlock.setVariable("direcao_alvo", diferencaAjustada);
                    
                    // Evaluate 
                    alvoFunctionBlock.evaluate();
                    
                    // OUTPUT
                    motor_esquerdo = (float)(alvoFunctionBlock.getVariable("motor_esquerdo").getValue() * 2);
                    motor_direito = (float)(alvoFunctionBlock.getVariable("motor_direito").getValue() * 2);
                
                } 
                
                // Chegou no Target fica parado
                if (destino_distanciaToTarget < 0.05){
                    motor_esquerdo = motor_direito = 0;
                }
                
                System.out.println("motor_esquerdo  = " + motor_esquerdo); 
                System.out.println("motor_direito = " + motor_direito);   
                
                
                // informa velocidade
                velocidadeMotores.getArray()[0] = motor_esquerdo * 2; //esquerda
                velocidadeMotores.getArray()[1] = motor_direito * 2; //direita
                vrep.simxCallScriptFunction(clientID, "K3_robot", vrep.sim_scripttype_childscript, "setVelocity", null, velocidadeMotores, null, null, null, null, null, null, vrep.simx_opmode_blocking);

            }
            
            //muito_perto the connection to V-REP:   
            vrep.simxFinish(clientID);
        }
        else
        	System.err.println("Erro ao conectar na API");
    }
}
