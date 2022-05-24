import gurobi.*;
import gurobi.GRB.IntParam;

import static gurobi.GRB.IntAttr.NumIntVars;
import static gurobi.GRB.IntAttr.VBasis;

import java.io.*;
import java.util.Arrays;

public class Gurobi {

    private static final int N_VERTICI = 43;
    private static final int LATI = (43*42/2);

    public static void main(String[] args) throws IOException {

        int[][] vertici = new int[903][3];

        parsing_file("src/coppia16.txt", vertici);

        try{

            GRBEnv env = new GRBEnv("Coppia16.log");

            env.set(IntParam.Presolve, 0);
            env.set(IntParam.Method, 0);

            GRBModel model = new GRBModel(env);

            //creazione variabili
            GRBVar[] link = new GRBVar[LATI];
            for(int l=0; l<LATI; l++) {
                link[l] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "X " + l);
            }

            GRBVar[] u = new GRBVar[N_VERTICI];
            for (int m=0; m<N_VERTICI; m++){
                u[m] = model.addVar(0.0, 43.0, 0.0, GRB.INTEGER, "u "+ u);
            }
            //---------------------------------------F.O.----------------------------------------

            GRBLinExpr expr = new GRBLinExpr();
            for(int k=0; k<LATI; k++) {
                expr.addTerm(vertici[k][2], link[k]);
            }
            model.setObjective(expr);
            model.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);

            //-------------------------------------VINCOLI---------------------------------------

            //vincoli per imporre l'attivazione di un solo link entrant e uscente
            for(int i=0; i<43; i++){
                expr = new GRBLinExpr();
                for(int j=43*i; j<43*i-i; j++){
                    //if(vertici[j][0] == i || vertici[j][1] == i)
                    expr.addTerm(1,link[j]);
                }
                model.addConstr(expr, GRB.EQUAL, 1, "una ed una sola entrata ed uscita per vertice");
            }

            //vincoli di Miller-Tucker-Zemil
            expr = new GRBLinExpr();
            //u[1] = 1
            expr.addTerm(1,u[1]);
            model.addConstr(expr, GRB.EQUAL, 1, "assegnazione u[1]");
            //2 <= u[i] <= n
            for (int i=2; i<N_VERTICI; i++){
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model.addConstr(expr, GRB.GREATER_EQUAL, 2, "limite inferiore di 2");
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model.addConstr(expr, GRB.LESS_EQUAL, N_VERTICI, "limite superiore di " + N_VERTICI);
            }


            model.optimize();

        }catch(GRBException e){
            System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }
    }

    public static void parsing_file(String file_name, int[][] vertici) throws IOException {

        File file = new File(file_name);
        BufferedReader br = new BufferedReader(new FileReader(file));
        String lettura;
        boolean start_parse = false;
        int i = 0;

        while((lettura = br.readLine()) != null){
            if(start_parse){
                int[] vertice = Arrays.stream(lettura.split(" ")).mapToInt(Integer::parseInt).toArray();
                vertici[i][0] = vertice[0];
                vertici[i][1] = vertice[1];
                vertici[i][2] = vertice[2];
                i++;
            }
            if(lettura.equals("Vertici 43")){
                start_parse = true;
            }
        }
    }
}
