import gurobi.*;
import gurobi.GRB.IntParam;

import java.io.*;
import java.util.Arrays;

public class Gurobi {

    private static final int N_VERTICI = 43;
    //private static final int LATI = (43*42/2);

    public static void main(String[] args) throws IOException {

        int[][] costi = new int[N_VERTICI][N_VERTICI];
        //int[][] vertici = new int[903][3];

        parsing_file("src/coppia16.txt", costi);

        try{

            GRBEnv env = new GRBEnv("Coppia16.log");

            env.set(IntParam.Presolve, 0);
            env.set(IntParam.Method, 0);

            GRBModel model = new GRBModel(env);

            //creazione variabili
            GRBVar[][] Xij = new GRBVar[N_VERTICI][N_VERTICI];
            for(int i=0; i< N_VERTICI; i++){
                for(int j=0; j< N_VERTICI; j++) {
                    Xij[i][j] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "X_" + i + "_" + j);
                }
            }

            GRBVar[] u = new GRBVar[N_VERTICI];
            for (int m=0; m<N_VERTICI; m++){
                u[m] = model.addVar(0.0, 43.0, 0.0, GRB.INTEGER, "u "+ u);
            }
            //---------------------------------------F.O.----------------------------------------

            GRBLinExpr expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(costi[i][j], Xij[i][j]);
                }
            }
            model.setObjective(expr);
            model.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);

            //-------------------------------------VINCOLI---------------------------------------

            //vincoli di assegnamento: per imporre l'attivazione di un solo link entrante e uscente

            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                for(int j=0; j<N_VERTICI; j++){
                    expr.addTerm(1, Xij[i][j]);
                }
                model.addConstr(expr, GRB.EQUAL, 1, "una sola uscita per ogni vertice");
            }

            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                for(int j=0; j<N_VERTICI; j++){
                    expr.addTerm(1, Xij[j][i]);
                }
                model.addConstr(expr, GRB.EQUAL, 1, "una sola entrata per ogni vertice");
            }

            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                expr.addTerm(1, Xij[i][i]);
                model.addConstr(expr, GRB.EQUAL, 0, "diagonale nulla");
            }


            //vincoli di Miller-Tucker-Zemil
            //u[1] = 1
            expr = new GRBLinExpr();
            expr.addTerm(1,u[1]);
            model.addConstr(expr, GRB.EQUAL, 1, "assegnazione u[1]");
            //2 <= u[i] <= n
            for (int i=2; i<N_VERTICI-2; i++){   //i = 1; i< N_VERTICI-1
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model.addConstr(expr, GRB.GREATER_EQUAL, 2, "limite inferiore di 2");
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model.addConstr(expr, GRB.LESS_EQUAL, N_VERTICI, "limite superiore di " + N_VERTICI);
            }
            //u[j] >= u[i] + 1 - (n-1)(1-xij) con i != j e i,j = 2...n
            //svolgendo il calcolo: u[j] >= u[i] + 2 - n + xij(n-1)
            for(int i=1; i<N_VERTICI-1; i++){
                for(int j=i+1; j<N_VERTICI; j++){
                    expr = new GRBLinExpr();
                    expr.addTerm(1,u[i]);
                    expr.addConstant(2-N_VERTICI);
                    expr.addTerm(N_VERTICI-1, Xij[i][j]);
                    model.addConstr(expr, GRB.GREATER_EQUAL, u[j], "");
                }
            }
            model.optimize();

            System.out.printf("funzione obiettivo = %f\n", model.get(GRB.DoubleAttr.ObjVal));
            for(GRBVar v: model.getVars())
                System.out.println(v.get(GRB.StringAttr.VarName) + ": " + v.get(GRB.DoubleAttr.X));

        }catch(GRBException e){
            System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }
    }

    public static void parsing_file(String file_name, int[][] costi) throws IOException {

        File file = new File(file_name);
        BufferedReader br = new BufferedReader(new FileReader(file));
        String lettura;
        boolean start_parse = false;

        while((lettura = br.readLine()) != null){
            if(start_parse){
                int[] numeri_letti = Arrays.stream(lettura.split(" ")).mapToInt(Integer::parseInt).toArray();
                costi[numeri_letti[0]][numeri_letti[1]] = costi[numeri_letti[1]][numeri_letti[0]]= numeri_letti[2];
            }
            if(lettura.equals("Vertici 43")){
                start_parse = true;
            }
        }
        //imposto la diagonale della matrice pari a 0; il costo da un nodo a sè stesso è pari a 0.
        for(int i= 0; i < N_VERTICI; i++){
            costi[i][i] = 0;
        }
    }
}
